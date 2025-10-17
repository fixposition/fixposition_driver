/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition sensor driver
 */

/* LIBC/STL */
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>

/* EXTERNAL */
#include <arpa/inet.h>
#include <fcntl.h>

#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/nmea.hpp>
#include <fpsdk_common/parser/novb.hpp>
#include <fpsdk_common/string.hpp>

/* PACKAGE */
#include "fixposition_driver_lib/fixposition_driver.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::common;
using namespace fpsdk::common::parser;
using namespace fpsdk::common::thread;

FixpositionDriver::FixpositionDriver(const DriverParams& params)
    : /* clang-format off */
    params_   { params },
    worker_   { "driver", std::bind(&FixpositionDriver::Worker, this) }  // clang-format on
{}

FixpositionDriver::~FixpositionDriver() { StopDriver(); }

// ---------------------------------------------------------------------------------------------------------------------

bool FixpositionDriver::Connect() {
    if (sensor_fd_ >= 0) {
        WARNING("Already connected to sensor");
        return true;
    }

    if (string::StrStartsWith(params_.stream_, "tcpcli://")) {
        const auto parts = string::StrSplit(params_.stream_.substr(9), ":");
        int port = 0;
        if ((parts.size() == 2) && !parts[0].empty() && !parts[1].empty() && string::StrToValue(parts[1], port)) {
            serial_ = false;
            INFO("Connecting to %s", params_.stream_.c_str());
            return ConnectTcp(parts[0], port);
        }
    } else if (string::StrStartsWith(params_.stream_, "serial://")) {
        const auto parts = string::StrSplit(params_.stream_.substr(9), ":");
        int baudrate = 0;
        if ((parts.size() == 2) && !parts[0].empty() && !parts[1].empty() && string::StrToValue(parts[1], baudrate)) {
            serial_ = true;
            INFO("Connecting to %s", params_.stream_.c_str());
            return ConnectSerial(parts[0], baudrate);
        }
    }

    WARNING("Bad stream spec: %s", params_.stream_.c_str());
    return false;
}

void FixpositionDriver::Disconnect() {
    if (IsConnected()) {
        INFO("Disconnecting from %s", params_.stream_.c_str());
        if (serial_) {
            DisconnectSerial();
        } else {
            DisconnectTcp();
        }
        sensor_fd_ = -1;
        params_.stream_.clear();
    }
}

bool FixpositionDriver::IsConnected() const { return sensor_fd_ >= 0; }

bool FixpositionDriver::ConnectTcp(const std::string& ip, const int port) {
    DisconnectTcp();

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        WARNING("Failed connecting to %s: %s", params_.stream_.c_str(), string::StrError(errno).c_str());
        return false;
    }

    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_addr.s_addr = inet_addr(ip.c_str());
    server_address.sin_port = htons(port);

    const int res = connect(fd, (struct sockaddr*)&server_address, sizeof server_address);

    if (res != 0) {
        WARNING("Failed connecting to %s: %s", params_.stream_.c_str(), string::StrError(errno).c_str());
        return false;
    }

    INFO("Connected to %s", params_.stream_.c_str());
    sensor_fd_ = fd;
    poll_fds_[0].fd = fd;
    poll_fds_[0].events = POLLIN | POLLERR | POLLHUP;
    return true;
}

void FixpositionDriver::DisconnectTcp() {
    if (sensor_fd_ >= 0) {
        close(sensor_fd_);
    }
}

#ifndef B460800
#define B460800 460800
#endif
#ifndef B500000
#define B500000 500000
#endif
#ifndef B921600
#define B921600 921600
#endif

bool FixpositionDriver::ConnectSerial(const std::string& dev, const int baud) {
    speed_t speed;
    switch (baud) {  // clang-format off
        case   9600: speed = B9600;    break;
        case  38400: speed = B38400;   break;
        case  57600: speed = B57600;   break;
        case 115200: speed = B115200;  break;
        case 230400: speed = B230400;  break;
        case 460800: speed = B460800;  break;
        case 500000: speed = B500000;  break;
        case 921600: speed = B921600;  break;  // clang-format on
        default:
            WARNING("Unsupported baudrate %d", baud);
            return false;
    }

    DisconnectSerial();

    int fd = open(dev.c_str(), O_RDWR | O_NOCTTY);

    if (fd == -1) {
        WARNING("Failed connecting to %s: %s", params_.stream_.c_str(), string::StrError(errno).c_str());
        return false;
    }

    // Get current serial port options:
    struct termios options;
    tcgetattr(fd, &options);
    options_save_ = options;

    char speed_buf[10];
    snprintf(speed_buf, sizeof(speed_buf), "0%06o", (int)cfgetispeed(&options));

    options.c_iflag &= ~(IXOFF | IXON | ICRNL);
    options.c_oflag &= ~(OPOST | ONLCR);
    options.c_lflag &= ~(ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN);
    options.c_cc[VEOL] = 0;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 50;

    cfsetospeed(&options, speed); /* baud rate */
    tcsetattr(fd, TCSANOW, &options);

    INFO("Connected to %s", params_.stream_.c_str());
    sensor_fd_ = fd;
    poll_fds_[0].fd = fd;
    poll_fds_[0].events = POLLIN | POLLERR | POLLHUP;
    return true;
}

void FixpositionDriver::DisconnectSerial() {
    if (sensor_fd_ >= 0) {
        tcsetattr(sensor_fd_, TCSANOW, &options_save_);
        close(sensor_fd_);
    }
}

std::size_t FixpositionDriver::Read(uint8_t* buf, const std::size_t size, const int timeout) {
    if (!IsConnected()) {
        WARNING_THR(1000, "no connection, cannot read");
        return 0;
    }

    const int res = poll(poll_fds_.data(), poll_fds_.size(), timeout);
    // Something's wrong
    if ((res < 0) && (errno != EINTR)) {
        WARNING("poll() fail: %s", string::StrError(errno).c_str());
    }
    // We should be able to read some data
    else if ((poll_fds_[0].revents & POLLIN) != 0) {
        ssize_t rv = 0;
        if (serial_) {
            rv = recv(sensor_fd_, buf, size, MSG_DONTWAIT);
        } else {
            rv = read(sensor_fd_, buf, sizeof(buf));
        }
        // We have some data
        if (rv >= 0) {
            return rv;
        }
        // Perhaps a problem
        else {
            if (errno == EAGAIN) {
                return 0;
            } else {
                WARNING("read/recv fail: %s", string::StrError(errno).c_str());
            }
        }
    }
    // Connection is broken
    else if ((poll_fds_[0].revents & (POLLHUP | POLLERR)) != 0) {
        WARNING("poll() fail");
    }
    // Timeout, no data at the moment
    else {
        return 0;
    }

    Disconnect();
    return 0;
}

bool FixpositionDriver::Write(const uint8_t* buf, const std::size_t size) {
    if (!IsConnected()) {
        WARNING_THR(1000, "no connection, cannot write");
        return 0;
    }

    int res = 0;
    if (serial_) {
        res = write(this->sensor_fd_, buf, size);
    } else {
        res = send(this->sensor_fd_, buf, size, MSG_DONTWAIT);
    }
    if (res < 0) {
        WARNING("send/write fail: %s", string::StrError(errno).c_str());
        return 0;
    }
    return (res > 0) && (res == (int)size);
}

// ---------------------------------------------------------------------------------------------------------------------

bool FixpositionDriver::StartDriver() { return Connect() && worker_.Start(); }

void FixpositionDriver::StopDriver() {
    if (worker_.GetStatus() == worker_.Status::RUNNING) {
        worker_.Stop();
    }
    Disconnect();
}

bool FixpositionDriver::Worker() {
    INFO("Driver running...");

    while (!thread.ShouldAbort()) {
        // While we're connected to the sensor...
        if (IsConnected()) {
            // Read more data from sensor and feed the parser
            uint8_t buf[parser::MAX_ADD_SIZE];
            const std::size_t size = Read(buf, sizeof(buf), 337);
            if (size == 0) {
                continue;  // try again
            }
            if (!parser_.Add(buf, size)) {
                WARNING("Parser overflow");  // should not happen, as we respect the parser's limits (MAX_ADD_SIZE)
                parser_.Reset();
                parser_.Add(buf, size);
            }

            // Process received message(s)
            parser::ParserMsg msg;
            while (parser_.Process(msg)) {
                IF_TRACE(msg.MakeInfo());
                TRACE("received %s %" PRIuMAX " -- %s", msg.name_.c_str(), msg.data_.size(), msg.info_.c_str());
                switch (msg.proto_) {
                    case parser::Protocol::FP_A:
                        NotifyFpaObservers(msg);
                        break;
                    case parser::Protocol::NMEA:
                        NotifyNmeaObservers(msg);
                        break;
                    case parser::Protocol::NOV_B:
                        NotifyNovbObservers(msg);
                        break;
                    case parser::Protocol::FP_B:
                    case parser::Protocol::UBX:
                    case parser::Protocol::RTCM3:
                    case parser::Protocol::UNI_B:
                    case parser::Protocol::SPARTN:
                    case parser::Protocol::OTHER:
                        break;
                }
                NotifyRawObservers(msg);
            }
        }
        // Reconnect after some time...
        else {
            INFO("Reconnecting in %.1f seconds...", params_.reconnect_delay_);
            if (worker_.Sleep(params_.reconnect_delay_ * 1000) == WaitRes::WOKEN) {
                break;
            }
            Connect();
        }
    }
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriver::AddFpaObserver(const std::string& message_name, FpaObserver observer) {
    DEBUG("Adding FP_A observer for %s", message_name.c_str());
    auto entry = fpa_observers_.find(message_name);
    if (entry == fpa_observers_.end()) {
        entry = fpa_observers_.insert({message_name, {}}).first;
    }
    entry->second.push_back(observer);
}

void FixpositionDriver::RemoveFpaObservers() { fpa_observers_.clear(); }

void FixpositionDriver::NotifyFpaObservers(const ParserMsg& msg) {
    auto entry = fpa_observers_.find(msg.name_);
    if ((entry != fpa_observers_.end()) && !entry->second.empty()) {
        TRACE("notify fpa %s", msg.name_.c_str());

#define _DISPATCH(_type_)                                                                                          \
    else if (msg.name_ == _type_::MSG_NAME) {                                                                      \
        _type_ payload;                                                                                            \
        if (payload.SetFromMsg(msg.data_.data(), msg.data_.size())) {                                              \
            for (auto& obs : entry->second) {                                                                      \
                obs(payload);                                                                                      \
            }                                                                                                      \
        } else {                                                                                                   \
            msg.MakeInfo();                                                                                        \
            WARNING_THR(1000, "Failed decoding %s: %s", msg.name_.c_str(), msg.info_.c_str());                     \
            TRACE_HEXDUMP(msg.data_.data(), msg.data_.size(), "    ", "Failed decoding %s: %s", msg.name_.c_str(), \
                          msg.info_.c_str());                                                                      \
        }                                                                                                          \
    }

        if (false) {
        }
        _DISPATCH(fpa::FpaEoePayload)
        _DISPATCH(fpa::FpaGnssantPayload)
        _DISPATCH(fpa::FpaGnsscorrPayload)
        _DISPATCH(fpa::FpaRawimuPayload)
        _DISPATCH(fpa::FpaCorrimuPayload)
        _DISPATCH(fpa::FpaImubiasPayload)
        _DISPATCH(fpa::FpaLlhPayload)
        _DISPATCH(fpa::FpaOdometryPayload)
        _DISPATCH(fpa::FpaOdomshPayload)
        _DISPATCH(fpa::FpaOdomenuPayload)
        _DISPATCH(fpa::FpaOdomstatusPayload)
        _DISPATCH(fpa::FpaTextPayload)
        _DISPATCH(fpa::FpaTfPayload)
        _DISPATCH(fpa::FpaTpPayload)

#undef _DISPATCH
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriver::AddNmeaObserver(const std::string& formatter, NmeaObserver observer) {
    DEBUG("Adding NMEA observer for %s", formatter.c_str());
    auto entry = nmea_observers_.find(formatter);
    if (entry == nmea_observers_.end()) {
        entry = nmea_observers_.insert({formatter, {}}).first;
    }
    entry->second.push_back(observer);
}

void FixpositionDriver::RemoveNmeaObservers() { nmea_observers_.clear(); }

void FixpositionDriver::NotifyNmeaObservers(const fpsdk::common::parser::ParserMsg& msg) {
    // NMEA observers are registered using the formatter (e.g. "RMC"), ignoring the talker ("GP", "GN", etc.)
    nmea::NmeaMessageMeta meta;
    if (!nmea::NmeaGetMessageMeta(meta, msg.data_.data(), msg.data_.size())) {
        WARNING_THR(1000, "Failed decoding %s", msg.name_.c_str());
        return;
    }

    auto entry = nmea_observers_.find(meta.formatter_);
    if ((entry != nmea_observers_.end()) && !entry->second.empty()) {
        TRACE("notify nmea %s (%s)", meta.formatter_, msg.name_.c_str());

#define _DISPATCH(_type_)                                                                                          \
    else if (std::strcmp(meta.formatter_, _type_::FORMATTER) == 0) {                                               \
        _type_ payload;                                                                                            \
        if (payload.SetFromMsg(msg.data_.data(), msg.data_.size())) {                                              \
            for (auto& obs : entry->second) {                                                                      \
                obs(payload);                                                                                      \
            }                                                                                                      \
        } else {                                                                                                   \
            msg.MakeInfo();                                                                                        \
            WARNING_THR(1000, "Bad %s: %s", msg.name_.c_str(), msg.info_.c_str());                                 \
            TRACE_HEXDUMP(msg.data_.data(), msg.data_.size(), "    ", "Failed decoding %s: %s", msg.name_.c_str(), \
                          msg.info_.c_str());                                                                      \
        }                                                                                                          \
    }

        if (false) {
        }
        _DISPATCH(nmea::NmeaGgaPayload)
        _DISPATCH(nmea::NmeaGllPayload)
        _DISPATCH(nmea::NmeaRmcPayload)
        _DISPATCH(nmea::NmeaVtgPayload)
        _DISPATCH(nmea::NmeaGstPayload)
        _DISPATCH(nmea::NmeaHdtPayload)
        _DISPATCH(nmea::NmeaZdaPayload)
        _DISPATCH(nmea::NmeaGsaPayload)
        _DISPATCH(nmea::NmeaGsvPayload)

#undef _DISPATCH
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriver::AddNovbObserver(const std::string& message_name, NovbObserver observer) {
    DEBUG("Adding NOV_B observer for %s", message_name.c_str());
    auto entry = novb_observers_.find(message_name);
    if (entry == novb_observers_.end()) {
        entry = novb_observers_.insert({message_name, {}}).first;
    }
    entry->second.push_back(observer);
}

void FixpositionDriver::RemoveNovbObservers() { novb_observers_.clear(); }

void FixpositionDriver::NotifyNovbObservers(const fpsdk::common::parser::ParserMsg& msg) {
    auto entry = novb_observers_.find(msg.name_);
    if ((entry != novb_observers_.end()) && !entry->second.empty()) {
        TRACE("notify novb %s", msg.name_.c_str());

        const uint8_t* frame = msg.data_.data();
        const novb::NovbHeader* header = (novb::NovbHeader*)&frame[0];
        const uint8_t* payload =
            (header->IsLongHeader() ? &frame[novb::NOV_B_HEAD_SIZE_LONG] : &frame[novb::NOV_B_HEAD_SIZE_SHORT]);
        for (auto& obs : entry->second) {
            obs(header, payload);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriver::AddRawObserver(RawObserver observer) {
    DEBUG("Adding observer for raw messages");
    raw_observers_.push_back(observer);
}

void FixpositionDriver::NotifyRawObservers(const fpsdk::common::parser::ParserMsg& msg) {
    if (!raw_observers_.empty()) {
        TRACE("notify raw %s", msg.name_.c_str());
        for (auto& obs : raw_observers_) {
            obs(msg);
        }
    }
}

void FixpositionDriver::RemoveRawObservers() { raw_observers_.clear(); }

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriver::SendCorrectionData(const uint8_t* msg, const std::size_t size) {
    // TODO: Check that RTCM message is valid. Maybe run the data through a Parser?
    TRACE("Send correction data (%" PRIuMAX " bytes)", size);
    if (!Write(msg, size)) {
        WARNING_THR(1000, "Failed sending correction data");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriver::SendWheelspeedData(const std::vector<WheelSpeedData>& data) {
    TRACE("Send wheelspeed data (#data=%" PRIuMAX ")", data.size());

    uint8_t payload[fpb::FP_B_MEASUREMENTS_HEAD_SIZE +
                    (fpb::FP_B_MEASUREMENTS_MAX_NUM_MEAS * fpb::FP_B_MEASUREMENTS_MEAS_SIZE)];

    std::size_t n_meas = 0;
    std::size_t payload_size = 0;
    bool meas_ok = true;
    for (auto& wheel : data) {
        fpb::FpbMeasurementsMeas meas = {0};

        meas.meas_type = types::EnumToVal(fpb::FpbMeasurementsMeasType::VELOCITY);
        // clang-format off
        if      (wheel.location_ == "RC") { meas.meas_loc = types::EnumToVal(parser::fpb::FpbMeasurementsMeasLoc::RC); }
        else if (wheel.location_ == "FR") { meas.meas_loc = types::EnumToVal(parser::fpb::FpbMeasurementsMeasLoc::FR); }
        else if (wheel.location_ == "FL") { meas.meas_loc = types::EnumToVal(parser::fpb::FpbMeasurementsMeasLoc::FL); }
        else if (wheel.location_ == "RR") { meas.meas_loc = types::EnumToVal(parser::fpb::FpbMeasurementsMeasLoc::RR); }
        else if (wheel.location_ == "RL") { meas.meas_loc = types::EnumToVal(parser::fpb::FpbMeasurementsMeasLoc::RL); }
        else                              { meas.meas_loc = types::EnumToVal(parser::fpb::FpbMeasurementsMeasLoc::UNSPECIFIED); meas_ok = false; }
        // clang-format on
        meas.timestamp_type = types::EnumToVal(fpb::FpbMeasurementsTimestampType::TIMEOFARRIVAL);
        meas.meas_x_valid = wheel.vx_valid_;
        meas.meas_x = wheel.vx_;
        meas.meas_y_valid = wheel.vy_valid_;
        meas.meas_y = wheel.vy_;
        meas.meas_z_valid = wheel.vz_valid_;
        meas.meas_z = wheel.vz_;

        std::memcpy(&payload[fpb::FP_B_MEASUREMENTS_HEAD_SIZE + (fpb::FP_B_MEASUREMENTS_MEAS_SIZE * n_meas)], &meas,
                    sizeof(meas));
        payload_size += sizeof(meas);

        n_meas++;
        if (n_meas >= fpb::FP_B_MEASUREMENTS_MAX_NUM_MEAS) {
            meas_ok = false;
            break;
        }
    }

    fpb::FpbMeasurementsHead head = {0};
    head.version = fpb::FP_B_MEASUREMENTS_V1;
    head.num_meas = n_meas;
    std::memcpy(&payload[0], &head, sizeof(head));
    payload_size += sizeof(head);

    if (n_meas == 0) {
        meas_ok = false;
    }
    if (!meas_ok) {
        WARNING_THR(1000, "Invalid WheelSpeedData (#data=%" PRIuMAX ", #meas=%" PRIuMAX ")", data.size(), n_meas);
    }

    std::vector<uint8_t> message;
    if (parser::fpb::FpbMakeMessage(message, parser::fpb::FP_B_MEASUREMENTS_MSGID, 0, payload, payload_size)) {
        if (!Write(message.data(), message.size())) {
            WARNING_THR(1000, "Failed sending FP_B-MEASUREMENTS message");
        }
    } else {
        WARNING_THR(1000, "Failed making FP_B-MEASUREMENTS message");
    }
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
