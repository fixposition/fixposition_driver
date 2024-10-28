/**
 *  @file
 *  @brief Implementation of FixpositionDriver class
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

/* SYSTEM / STL */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdexcept>

/* PACKAGE */
#include <fixposition_driver_lib/fixposition_driver.hpp>
#include <fixposition_driver_lib/parser.hpp>

#ifndef B460800
#define B460800 460800
#endif
#ifndef B500000
#define B500000 500000
#endif
#ifndef B921600
#define B921600 921600
#endif
#ifndef B1000000
#define B1000000 1000000
#endif

namespace fixposition {
FixpositionDriver::FixpositionDriver(const FixpositionDriverParams& params) : params_(params) {
    // connect to the sensor
    if (!Connect()) {
        if (params_.fp_output.type == INPUT_TYPE::TCP) {
            throw std::runtime_error("Unable to connect to the sensor via TCP");
        } else if (params_.fp_output.type == INPUT_TYPE::SERIAL) {
            throw std::runtime_error("Unable to connect to the sensor via Serial");
        } else {
            throw std::runtime_error("Unable to connect to the sensor, verify configuration");
        }
    }

    // initialize converters
    if (!InitializeConverters()) {
        throw std::runtime_error("Could not initialize output converter!");
    }
}

FixpositionDriver::~FixpositionDriver() {
    if (client_fd_ != -1) {
        if (params_.fp_output.type == INPUT_TYPE::SERIAL) {
            tcsetattr(client_fd_, TCSANOW, &options_save_);
        }
        close(client_fd_);
    }
}

bool FixpositionDriver::Connect() {
    switch (params_.fp_output.type) {
        case INPUT_TYPE::TCP:
            return CreateTCPSocket();
            break;
        case INPUT_TYPE::SERIAL:
            return CreateSerialConnection();
            break;
        default:
            std::cerr << "Unknown connection type!\n";
            return false;
    }
}

void FixpositionDriver::WsCallback(
    const std::unordered_map<std::string, std::vector<std::pair<bool, int>>>& sensors_meas) {
    std::vector<FpbMeasurementsMeas> sensor_measurements;
    for (const auto& sensor : sensors_meas) {
        const FpbMeasurementsMeasLoc location = WsMeasStringToLoc(sensor.first);
        if (location == MEASLOC_UNSPECIFIED) {
            std::cerr << "Unknown measurement type will not be processed!\n";
            continue;
        }
        FpbMeasurementsMeas fpb_meas;
        if (FillWsSensorMeas(sensor.second, location, fpb_meas)) {
            sensor_measurements.push_back(fpb_meas);
        }
    }

    const size_t num_meas = sensor_measurements.size();
    if (num_meas == 0 || num_meas > 10) {
        std::cerr << "Number of wheel speed sensors is invalid.\n";
        return;
    }

    FpbHeader header;
    header.sync1 = 0x66;
    header.sync2 = 0x21;
    header.msg_id = 2001;
    header.payload_size = FP_B_MEASUREMENTS_HEAD_SIZE + (FP_B_MEASUREMENTS_BODY_SIZE * num_meas);
    header.time = 0;

    FpbMeasurementsHeader meas_header;
    meas_header.version = 1;
    meas_header.num_meas = num_meas;
    std::fill(&meas_header.reserved0[0], &meas_header.reserved0[6], 0);

    const int msg_sz =
        FP_B_HEAD_SIZE + FP_B_MEASUREMENTS_HEAD_SIZE + (FP_B_MEASUREMENTS_BODY_SIZE * num_meas) + FP_B_CRC_SIZE;
    std::vector<uint8_t> message(msg_sz);

    memcpy(&message[0], (uint8_t*)&header, sizeof(header));
    memcpy(&message[FP_B_HEAD_SIZE], (uint8_t*)&meas_header, sizeof(meas_header));
    for (size_t i = 0; i < num_meas; i++) {
        memcpy(&message[FP_B_HEAD_SIZE + FP_B_MEASUREMENTS_HEAD_SIZE + (FP_B_MEASUREMENTS_BODY_SIZE * i)],
               (uint8_t*)&sensor_measurements[i], sizeof(sensor_measurements[i]));
    }
    const uint32_t crc = Crc32fpb(
        message.data(), FP_B_HEAD_SIZE + FP_B_MEASUREMENTS_HEAD_SIZE + (FP_B_MEASUREMENTS_BODY_SIZE * num_meas));
    memcpy(&message[FP_B_HEAD_SIZE + FP_B_MEASUREMENTS_HEAD_SIZE + (FP_B_MEASUREMENTS_BODY_SIZE * num_meas)], &crc,
           sizeof(crc));

    switch (params_.fp_output.type) {
        case INPUT_TYPE::TCP:
            send(this->client_fd_, &message[0], message.size(), MSG_DONTWAIT);
            break;
        case INPUT_TYPE::SERIAL:
            (void)!write(this->client_fd_, &message[0], message.size());
            // Suppress warning - https://stackoverflow.com/a/64407070/7944565
            break;
        default:
            std::cerr << "Unknown connection type!\n";
            break;
    }
}

void FixpositionDriver::RtcmCallback(const void *rtcm_msg, const size_t msg_size) {
    // TODO: Check that RTCM message is valid

    switch (params_.fp_output.type) {
        case INPUT_TYPE::TCP:
            send(this->client_fd_, rtcm_msg, msg_size, MSG_DONTWAIT);
            break;
        case INPUT_TYPE::SERIAL:
            (void)!write(this->client_fd_, rtcm_msg, msg_size);
            // Suppress warning - https://stackoverflow.com/a/64407070/7944565
            break;
        default:
            std::cerr << "Unknown connection type!\n";
            break;
    }
}

bool FixpositionDriver::FillWsSensorMeas(const std::vector<std::pair<bool, int>>& meas_vec,
                                         const FpbMeasurementsMeasLoc meas_loc, FpbMeasurementsMeas& meas_fpb) {
    const size_t num_axis = meas_vec.size();
    if (num_axis != 3) {
        std::cerr << "Wheelspeed sensor has an invalid number of measurements.\n";
        return false;
    }
    meas_fpb.meas_type = MEASTYPE_VELOCITY;
    meas_fpb.meas_loc = meas_loc;
    std::fill(&meas_fpb.reserved1[0], &meas_fpb.reserved1[4], 0);
    // In the current setup, the sensor will handle the timestamping as time of arrival.
    meas_fpb.timestamp_type = TIME_TOA;
    meas_fpb.gps_wno = 0;
    meas_fpb.gps_tow = 0;
    meas_fpb.meas_x_valid = meas_vec[0].first;
    meas_fpb.meas_x = meas_vec[0].second;
    meas_fpb.meas_y_valid = meas_vec[1].first;
    meas_fpb.meas_y = meas_vec[1].second;
    meas_fpb.meas_z_valid = meas_vec[2].first;
    meas_fpb.meas_z = meas_vec[2].second;
    return true;
}

FpbMeasurementsMeasLoc FixpositionDriver::WsMeasStringToLoc(const std::string& meas_loc) {
    if (meas_loc == "RC") return MEASLOC_RC;
    if (meas_loc == "FR") return MEASLOC_FR;
    if (meas_loc == "FL") return MEASLOC_FL;
    if (meas_loc == "RR") return MEASLOC_RR;
    if (meas_loc == "RL") return MEASLOC_RL;
    return MEASLOC_UNSPECIFIED;
}

bool FixpositionDriver::InitializeConverters() {
    for (const auto& format : params_.fp_output.formats) {
        
        // FP_A messages
        if (format == "ODOMETRY") {
            a_converters_["ODOMETRY"] = std::unique_ptr<NmeaConverter<FP_ODOMETRY>>(new NmeaConverter<FP_ODOMETRY>());
        } else if (format == "ODOMENU") {
            a_converters_["ODOMENU"] = std::unique_ptr<NmeaConverter<FP_ODOMENU>>(new NmeaConverter<FP_ODOMENU>());
        } else if (format == "ODOMSH") {
            a_converters_["ODOMSH"] = std::unique_ptr<NmeaConverter<FP_ODOMSH>>(new NmeaConverter<FP_ODOMSH>());
        } else if (format == "ODOMSTATUS") {
            a_converters_["ODOMSTATUS"] = std::unique_ptr<NmeaConverter<FP_ODOMSTATUS>>(new NmeaConverter<FP_ODOMSTATUS>());
        } else if (format == "LLH") {
            a_converters_["LLH"] = std::unique_ptr<NmeaConverter<FP_LLH>>(new NmeaConverter<FP_LLH>());
        } else if (format == "TF") {
            a_converters_["TF"] = std::unique_ptr<NmeaConverter<FP_TF>>(new NmeaConverter<FP_TF>());
        } else if (format == "TP") {
            a_converters_["TP"] = std::unique_ptr<NmeaConverter<FP_TP>>(new NmeaConverter<FP_TP>());
        } else if (format == "RAWIMU") {
            a_converters_["RAWIMU"] = std::unique_ptr<NmeaConverter<FP_RAWIMU>>(new NmeaConverter<FP_RAWIMU>());
        } else if (format == "CORRIMU") {
            a_converters_["CORRIMU"] = std::unique_ptr<NmeaConverter<FP_CORRIMU>>(new NmeaConverter<FP_CORRIMU>());
        } else if (format == "IMUBIAS") {
            a_converters_["IMUBIAS"] = std::unique_ptr<NmeaConverter<FP_IMUBIAS>>(new NmeaConverter<FP_IMUBIAS>());
        } else if (format == "GNSSANT") {
            a_converters_["GNSSANT"] = std::unique_ptr<NmeaConverter<FP_GNSSANT>>(new NmeaConverter<FP_GNSSANT>());
        } else if (format == "GNSSCORR") {
            a_converters_["GNSSCORR"] = std::unique_ptr<NmeaConverter<FP_GNSSCORR>>(new NmeaConverter<FP_GNSSCORR>());
        } else if (format == "TEXT") {
            a_converters_["TEXT"] = std::unique_ptr<NmeaConverter<FP_TEXT>>(new NmeaConverter<FP_TEXT>());
        } else if (format == "EOE") {
            a_converters_["EOE"] = std::unique_ptr<NmeaConverter<FP_EOE>>(new NmeaConverter<FP_EOE>());
        
        // NMEA messages
        } else if (format == "GPGGA") {
            a_converters_["GPGGA"] = std::unique_ptr<NmeaConverter<GP_GGA>>(new NmeaConverter<GP_GGA>());
        } else if (format == "GPGLL") {
            a_converters_["GPGLL"] = std::unique_ptr<NmeaConverter<GP_GLL>>(new NmeaConverter<GP_GLL>());
        } else if (format == "GNGSA") {
            a_converters_["GNGSA"] = std::unique_ptr<NmeaConverter<GN_GSA>>(new NmeaConverter<GN_GSA>());
        } else if (format == "GPGST") {
            a_converters_["GPGST"] = std::unique_ptr<NmeaConverter<GP_GST>>(new NmeaConverter<GP_GST>());
        } else if (format == "GXGSV") {
            a_converters_["GXGSV"] = std::unique_ptr<NmeaConverter<GX_GSV>>(new NmeaConverter<GX_GSV>());
        } else if (format == "GPHDT") {
            a_converters_["GPHDT"] = std::unique_ptr<NmeaConverter<GP_HDT>>(new NmeaConverter<GP_HDT>());
        } else if (format == "GPRMC") {
            a_converters_["GPRMC"] = std::unique_ptr<NmeaConverter<GP_RMC>>(new NmeaConverter<GP_RMC>());
        } else if (format == "GPVTG") {
            a_converters_["GPVTG"] = std::unique_ptr<NmeaConverter<GP_VTG>>(new NmeaConverter<GP_VTG>());
        } else if (format == "GPZDA") {
            a_converters_["GPZDA"] = std::unique_ptr<NmeaConverter<GP_ZDA>>(new NmeaConverter<GP_ZDA>());
        } else {
            std::cerr << "Unknown input format: " << format << "\n";
        }
    }
    return !a_converters_.empty();
}
bool FixpositionDriver::RunOnce() {
    if ((client_fd_ > 0) && (connection_status_ == 0) && ReadAndPublish()) {
        return true;
    } else {
        close(client_fd_);
        client_fd_ = -1;
        return false;
    }
}

bool FixpositionDriver::ReadAndPublish() {
    int msg_size = 0;
    // Nov B
    msg_size = IsNovMessage(readBuf, buf_size);
    if (msg_size > 0) {
        NovConvertAndPublish(readBuf);
        buf_size -= msg_size;
        if (buf_size > 0) {
            memmove(readBuf, &readBuf[msg_size], buf_size);
        }
        return true;
    } else if (msg_size == 0) {
        // Nmea (incl. FP_A)
        msg_size = IsNmeaMessage((char*)readBuf, buf_size);
        if (msg_size > 0) {
            NmeaConvertAndPublish({(const char*)readBuf, (const char*)readBuf + msg_size});
            buf_size -= msg_size;
            if (buf_size > 0) {
                memmove(readBuf, &readBuf[msg_size], buf_size);
            }
            return true;
        } else if (msg_size == 0) {
            // If not NOV_B nor NMEA, remove 1 char
            if (buf_size > 0) {
                buf_size -= 1;
                memmove(readBuf, &readBuf[1], buf_size);
            }
        } else {
            // Wait for more data
        }
    } else {
        // wait for more data
    }

    // Read more data from the TCP/Serial port
    int rem_size = sizeof(readBuf) - buf_size;
    if (rem_size > 0) {
        ssize_t rv;
        if (params_.fp_output.type == INPUT_TYPE::TCP) {
            rv = recv(client_fd_, (void*)&readBuf[buf_size], sizeof(readBuf) - buf_size, MSG_DONTWAIT);
        } else if (params_.fp_output.type == INPUT_TYPE::SERIAL) {
            rv = read(client_fd_, (void*)&readBuf[buf_size], sizeof(readBuf) - buf_size);
        } else {
            rv = 0;
        }

        if (rv == 0) {
            std::cerr << "Connection closed.\n";
            return false;
        }
        
        if (rv < 0 && errno == EAGAIN) {
            /* no data for now, call back when the socket is readable */
            return true;
        } 
        
        if (rv < 0) {
            std::cerr << "Connection error.\n";
            return false;
        }

        buf_size += rv;
    }

    return true;
}

void FixpositionDriver::NmeaConvertAndPublish(const std::string& msg) {
    // split the msg into tokens, removing the *XX checksum
    std::vector<std::string> tokens;
    std::size_t star_pos = msg.find_last_of("*");
    SplitMessage(tokens, msg.substr(1, star_pos - 1), ",");

    // if it doesn't start with FP then do nothing
    if ((tokens.at(0) != "FP") && (tokens.at(0) != "GPGGA") && 
        (tokens.at(0) != "GPGLL") && (tokens.at(0) != "GNGSA") &&
        (tokens.at(0) != "GPGST") && (tokens.at(0) != "GPHDT") &&
        (tokens.at(0) != "GPRMC") && (tokens.at(0) != "GPVTG") &&
        (tokens.at(0) != "GPZDA") && (tokens.at(0) != "GPGSV") &&
        (tokens.at(0) != "GAGSV") && (tokens.at(0) != "GBGSV") &&
        (tokens.at(0) != "GLGSV")) {
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
    } else if (tokens.at(0) == "GPGSV" || tokens.at(0) == "GAGSV" || tokens.at(0) == "GBGSV" || tokens.at(0) == "GLGSV") {
        _header = "GXGSV";
    } else {
        _header = tokens.at(1);
    }
    const std::string header = _header;

    // If we have a converter available, convert to ros.
    if (a_converters_[header] != nullptr) {
        a_converters_[header]->ConvertTokens(tokens);
    }
}

void FixpositionDriver::NovConvertAndPublish(const uint8_t* msg) {
    auto* header = reinterpret_cast<const Oem7MessageHeaderMem*>(msg);
    const auto msg_id = header->message_id;

    if (msg_id == static_cast<uint16_t>(MessageId::BESTGNSSPOS)) {
        for (auto& ob : bestgnsspos_obs_) {
            auto* payload = reinterpret_cast<const BESTGNSSPOSMem*>(msg + sizeof(Oem7MessageHeaderMem));
            ob(header, payload);
        }
    }
    // TODO add more msg types
}

bool FixpositionDriver::CreateTCPSocket() {
    if (client_fd_ != -1) {
        std::cerr << "TCP connection already exists.\n";
        return true;
    }

    client_fd_ = socket(AF_INET, SOCK_STREAM, 0);

    if (client_fd_ < 0) {
        std::cerr << "Error in client creation.\n";
        return false;
    } else {
        std::cout << "Client created.\n";
    }

    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(std::stoi(params_.fp_output.port));
    server_address.sin_addr.s_addr = inet_addr(params_.fp_output.ip.c_str());

    connection_status_ = connect(client_fd_, (struct sockaddr*)&server_address, sizeof server_address);

    if (connection_status_ != 0) {
        std::cerr << "Error on connection of TCP socket: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}

bool FixpositionDriver::CreateSerialConnection() {
    if (client_fd_ != -1) {
        std::cerr << "Serial connection already exists.\n";
        return true;
    }

    client_fd_ = open(params_.fp_output.port.c_str(), O_RDWR | O_NOCTTY);

    struct termios options;
    speed_t speed;

    switch (params_.fp_output.baudrate) {
        case 9600:
            speed = B9600;
            break;

        case 38400:
            speed = B38400;
            break;

        case 57600:
            speed = B57600;
            break;

        case 115200:
            speed = B115200;
            break;

        case 230400:
            speed = B230400;
            break;

        case 460800:
            speed = B460800;
            break;

        case 500000:
            speed = B500000;
            break;

        case 921600:
            speed = B921600;
            break;

        case 1000000:
            speed = B1000000;
            break;

        default:
            speed = B115200;
            std::cerr << "Unsupported baudrate: " << params_.fp_output.baudrate
                      << "\n\tsupported examples:\n\t9600, "
                         "19200, "
                         "38400, "
                         "57600\t\n115200\n230400\n460800\n500000\n921600\n1000000\n";
    }

    if (client_fd_ == -1) {
        // Could not open the port.
        std::cerr << "Failed to open serial port " << strerror(errno) << "\n";
        return false;
    } else {
        // Get current serial port options:
        tcgetattr(client_fd_, &options);
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
        tcsetattr(client_fd_, TCSANOW, &options);
        connection_status_ = 0;  // not used for serial, set to 0 (success)
        return true;
    }
}
}  // namespace fixposition
