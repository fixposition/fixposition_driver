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
 * @brief Implementation of FixpositionDriver class
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

FixpositionDriver::FixpositionDriver(const SensorParams& params)
    : /* clang-format off */
    params_   { params },
    worker_   { "driver", std::bind(&FixpositionDriver::Worker, this, std::placeholders::_1) }  // clang-format on
{
    // Initialize converters based on config
    bool ok = true;
    for (const auto& format : params_.formats) {
        // clang-format off
        // FP_A messages
        if      (format == "ODOMETRY")     { a_converters_["ODOMETRY"]     = std::make_unique<NmeaConverter<FP_ODOMETRY>>();     }
        else if (format == "ODOMENU")      { a_converters_["ODOMENU"]      = std::make_unique<NmeaConverter<FP_ODOMENU>>();      }
        else if (format == "ODOMSH")       { a_converters_["ODOMSH"]       = std::make_unique<NmeaConverter<FP_ODOMSH>>();       }
        else if (format == "ODOMSTATUS")   { a_converters_["ODOMSTATUS"]   = std::make_unique<NmeaConverter<FP_ODOMSTATUS>>();   }
        else if (format == "LLH")          { a_converters_["LLH"]          = std::make_unique<NmeaConverter<FP_LLH>>();          }
        else if (format == "TF")           { a_converters_["TF"]           = std::make_unique<NmeaConverter<FP_TF>>();           }
        else if (format == "TP")           { a_converters_["TP"]           = std::make_unique<NmeaConverter<FP_TP>>();           }
        else if (format == "RAWIMU")       { a_converters_["RAWIMU"]       = std::make_unique<NmeaConverter<FP_RAWIMU>>();       }
        else if (format == "CORRIMU")      { a_converters_["CORRIMU"]      = std::make_unique<NmeaConverter<FP_CORRIMU>>();      }
        else if (format == "IMUBIAS")      { a_converters_["IMUBIAS"]      = std::make_unique<NmeaConverter<FP_IMUBIAS>>();      }
        else if (format == "GNSSANT")      { a_converters_["GNSSANT"]      = std::make_unique<NmeaConverter<FP_GNSSANT>>();      }
        else if (format == "GNSSCORR")     { a_converters_["GNSSCORR"]     = std::make_unique<NmeaConverter<FP_GNSSCORR>>();     }
        else if (format == "TEXT")         { a_converters_["TEXT"]         = std::make_unique<NmeaConverter<FP_TEXT>>();         }
        else if (format == "EOE")          { a_converters_["EOE"]          = std::make_unique<NmeaConverter<FP_EOE>>();          }
        // NMEA messages
        else if (format == "GPGGA")        { a_converters_["GPGGA"]        = std::make_unique<NmeaConverter<GP_GGA>>();          }
        else if (format == "GPGLL")        { a_converters_["GPGLL"]        = std::make_unique<NmeaConverter<GP_GLL>>();          }
        else if (format == "GNGSA")        { a_converters_["GNGSA"]        = std::make_unique<NmeaConverter<GN_GSA>>();          }
        else if (format == "GPGST")        { a_converters_["GPGST"]        = std::make_unique<NmeaConverter<GP_GST>>();          }
        else if (format == "GXGSV")        { a_converters_["GXGSV"]        = std::make_unique<NmeaConverter<GX_GSV>>();          }
        else if (format == "GPHDT")        { a_converters_["GPHDT"]        = std::make_unique<NmeaConverter<GP_HDT>>();          }
        else if (format == "GPRMC")        { a_converters_["GPRMC"]        = std::make_unique<NmeaConverter<GP_RMC>>();          }
        else if (format == "GPVTG")        { a_converters_["GPVTG"]        = std::make_unique<NmeaConverter<GP_VTG>>();          }
        else if (format == "GPZDA")        { a_converters_["GPZDA"]        = std::make_unique<NmeaConverter<GP_ZDA>>();          }
        // clang-format on
        else {
            WARNING("Unknown input format '%s'", format.c_str());
        }
    }
    if (a_converters_.empty()) {
        WARNING("No converter formats defined");
    }
    if (!ok) {
        throw std::runtime_error("Could not initialize output converter(s)");
    }
}

FixpositionDriver::~FixpositionDriver() { StopDriver(); }

// ---------------------------------------------------------------------------------------------------------------------

bool FixpositionDriver::Connect() {
    if (sensor_fd_ >= 0) {
        WARNING("Already connected to sensor");
        return true;
    }

    // Work out connection type. A bit of a hack, but this will soon be replaced by fpsdk::common::stream anyways...
    const auto parts = string::StrSplit(params_.stream_, ":");
    int port_or_baudrate = 0;
    if ((parts.size() != 2) || parts[0].empty() || parts[1].empty() ||
        !string::StrToValue(parts[1], port_or_baudrate)) {
        WARNING("Bad sensor stream spec: %s", params_.stream_.c_str());
        return false;
    }
    const std::string& ip_or_dev = parts[0];
    serial_ = (parts[0].substr(0, 1) == "/");

    INFO("Connecting to %s", params_.stream_.c_str());

    if (serial_) {
        return ConnectSerial(ip_or_dev, port_or_baudrate);
    } else {
        return ConnectTcp(ip_or_dev, port_or_baudrate);
    }
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
    worker_.Stop();
    Disconnect();
}

void FixpositionDriver::Worker(void* /*arg*/) {
    INFO("Driver running...");

    while (!worker_.ShouldAbort()) {
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
                TRACE("received %s %" PRIuMAX " %s", msg.name_.c_str(), msg.data_.size(), msg.info_.c_str());
                switch (msg.proto_) {
                    case parser::Protocol::FP_A:
                        ProcessFpa(msg);
                        break;
                    case parser::Protocol::NMEA:
                        ProcessNmea(msg);
                        break;
                    case parser::Protocol::NOV_B:
                        ProcessNovb(msg);
                        break;
                    case parser::Protocol::FP_B:
                    case parser::Protocol::UBX:
                    case parser::Protocol::RTCM3:
                    case parser::Protocol::UNI_B:
                    case parser::Protocol::SPARTN:
                    case parser::Protocol::OTHER:
                        break;
                }
            }
        }
        // Reconnect after some time...
        else {
            INFO("Reconnecting in %.1f seconds...", params_.reconnect_delay_);
            if (worker_.Sleep(params_.reconnect_delay_ * 1000)) {
                break;
            }
            Connect();
        }
    }
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

void FixpositionDriver::ProcessFpa(const ParserMsg& msg) {
    auto entry = fpa_observers_.find(msg.name_);
    if ((entry != fpa_observers_.end()) && !entry->second.empty()) {
        TRACE("process %s", msg.name_.c_str());

#define _DISPATCH(_type_)                                             \
    else if (msg.name_ == _type_::MSG_NAME) {                         \
        _type_ payload;                                               \
        if (payload.SetFromMsg(msg.data_.data(), msg.data_.size())) { \
            for (auto& obs : entry->second) {                         \
                obs(payload);                                         \
            }                                                         \
        } else {                                                      \
            WARNING_THR(1000, "Bad %s", msg.name_.c_str());           \
        }                                                             \
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
    } else {
        TRACE("ignore %s", msg.name_.c_str());
    }

    NmeaConvertAndPublish(msg);  // TODO: replace this
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

void FixpositionDriver::ProcessNmea(const fpsdk::common::parser::ParserMsg& msg) {
    // NMEA observers are registered using the formatter (e.g. "RMC"), ignoring the talker ("GP", "GN", etc.)
    nmea::NmeaMessageMeta meta;
    if (!nmea::NmeaGetMessageMeta(meta, msg.data_.data(), msg.data_.size())) {
        WARNING_THR(1000, "Bad %s", msg.name_.c_str());
        return;
    }

    auto entry = nmea_observers_.find(meta.formatter_);
    if ((entry != nmea_observers_.end()) && !entry->second.empty()) {
        TRACE("process %s (%s)", meta.formatter_, msg.name_.c_str());

#define _DISPATCH(_type_)                                             \
    else if (std::strcmp(meta.formatter_, _type_::FORMATTER) == 0) {  \
        _type_ payload;                                               \
        if (payload.SetFromMsg(msg.data_.data(), msg.data_.size())) { \
            for (auto& obs : entry->second) {                         \
                obs(payload);                                         \
            }                                                         \
        } else {                                                      \
            WARNING_THR(1000, "Bad %s", msg.name_.c_str());           \
        }                                                             \
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
    } else {
        TRACE("ignore %s (%s)", meta.formatter_, msg.name_.c_str());
    }

    NmeaConvertAndPublish(msg);  // TODO: replace this
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

void FixpositionDriver::ProcessNovb(const fpsdk::common::parser::ParserMsg& msg) {
    auto entry = novb_observers_.find(msg.name_);
    if ((entry != novb_observers_.end()) && !entry->second.empty()) {
        TRACE("process %s", msg.name_.c_str());
        for (auto& obs : entry->second) {
            obs(msg);
        }
    } else {
        TRACE("ignore %s", msg.name_.c_str());
    }

    NovConvertAndPublish(msg);  // TODO: replace this
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriver::NmeaConvertAndPublish(const fpsdk::common::parser::ParserMsg& msg) {
    const std::string sentence = {(const char*)msg.data_.data(), (const char*)msg.data_.data() + msg.data_.size()};
    // split the msg into tokens, removing the *XX checksum
    const std::size_t star_pos = sentence.find_last_of("*");
    const std::vector<std::string> tokens = string::StrSplit(sentence.substr(1, star_pos - 1), ",");

    // if it doesn't start with FP then do nothing
    if ((tokens.at(0) != "FP") && (tokens.at(0) != "GPGGA") && (tokens.at(0) != "GPGLL") && (tokens.at(0) != "GNGSA") &&
        (tokens.at(0) != "GPGST") && (tokens.at(0) != "GPHDT") && (tokens.at(0) != "GPRMC") &&
        (tokens.at(0) != "GPVTG") && (tokens.at(0) != "GPZDA") && (tokens.at(0) != "GPGSV") &&
        (tokens.at(0) != "GAGSV") && (tokens.at(0) != "GBGSV") && (tokens.at(0) != "GLGSV")) {
        return;
    }

    // Get the header of the sentence
    std::string _header;
    if (tokens.at(0) == "GPGGA") {
        _header = "GPGGA";
    } else if (tokens.at(0) == "GPGLL") {
        _header = "GPGLL";
    } else if (tokens.at(0) == "GNGSA") {
        _header = "GNGSA";
    } else if (tokens.at(0) == "GPGST") {
        _header = "GPGST";
    } else if (tokens.at(0) == "GPHDT") {
        _header = "GPHDT";
    } else if (tokens.at(0) == "GPRMC") {
        _header = "GPRMC";
    } else if (tokens.at(0) == "GPVTG") {
        _header = "GPVTG";
    } else if (tokens.at(0) == "GPZDA") {
        _header = "GPZDA";
    } else if (tokens.at(0) == "GPGSV" || tokens.at(0) == "GAGSV" || tokens.at(0) == "GBGSV" ||
               tokens.at(0) == "GLGSV") {
        _header = "GXGSV";
    } else {
        _header = tokens.at(1);
    }
    const std::string header = _header;

    // If we have a converter available, convert to ROS msg
    if (a_converters_[header] != nullptr) {
        a_converters_[header]->ConvertTokens(tokens);
    }
}

void FixpositionDriver::NovConvertAndPublish(const fpsdk::common::parser::ParserMsg& msg) {
    auto* header = reinterpret_cast<const parser::novb::NovbLongHeader*>(msg.data_.data());
    const uint16_t msg_id = header->message_id;

    if (msg_id == parser::novb::NOV_B_BESTGNSSPOS_MSGID) {
        for (auto& ob : bestgnsspos_obs_) {
            auto* payload = reinterpret_cast<const parser::novb::NovbBestgnsspos*>(
                msg.data_.data() + sizeof(parser::novb::NovbLongHeader));
            ob(header, payload);
        }
    }
    // TODO add more msg types
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriver::WsCallback(
    const std::unordered_map<std::string, std::vector<std::pair<bool, int>>>& sensors_meas) {
    std::vector<parser::fpb::FpbMeasurementsMeas> sensor_measurements;
    for (const auto& sensor : sensors_meas) {
        const parser::fpb::FpbMeasurementsMeasLoc location = WsMeasStringToLoc(sensor.first);
        if (location == parser::fpb::FpbMeasurementsMeasLoc::UNSPECIFIED) {
            WARNING("Unknown measurement type will not be processed");
            continue;
        }
        parser::fpb::FpbMeasurementsMeas fpb_meas;
        if (FillWsSensorMeas(sensor.second, location, fpb_meas)) {
            sensor_measurements.push_back(fpb_meas);
        }
    }

    const size_t num_meas = sensor_measurements.size();
    if ((num_meas == 0) || (num_meas > parser::fpb::FP_B_MEASUREMENTS_MAX_NUM_MEAS)) {
        WARNING("Number of wheel speed sensors is invalid");
        return;
    }

    uint8_t payload[parser::fpb::FP_B_MEASUREMENTS_HEAD_SIZE +
                    (parser::fpb::FP_B_MEASUREMENTS_MAX_NUM_MEAS * parser::fpb::FP_B_MEASUREMENTS_MEAS_SIZE)];
    const std::size_t payload_size = parser::fpb::FP_B_MEASUREMENTS_HEAD_SIZE +
                                     (sensor_measurements.size() * parser::fpb::FP_B_MEASUREMENTS_MEAS_SIZE);

    parser::fpb::FpbMeasurementsHead meas_header;
    meas_header.version = parser::fpb::FP_B_MEASUREMENTS_V1;
    meas_header.num_meas = num_meas;
    std::memcpy(&payload[0], &meas_header, sizeof(meas_header));

    for (std::size_t i = 0; i < num_meas; i++) {
        memcpy(&payload[parser::fpb::FP_B_MEASUREMENTS_HEAD_SIZE + (parser::fpb::FP_B_MEASUREMENTS_MEAS_SIZE * i)],
               &sensor_measurements[i], sizeof(sensor_measurements[i]));
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

void FixpositionDriver::RtcmCallback(const uint8_t* rtcm_msg, const size_t msg_size) {
    // TODO: Check that RTCM message is valid

    if (!Write(rtcm_msg, msg_size)) {
        WARNING_THR(1000, "Failed sending XXX message");
    }
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
