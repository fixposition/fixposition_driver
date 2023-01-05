/**
 *  @file
 *  @brief Implementation of FixpositionDriver class
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* SYSTEM / STL */
#include <chrono>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>

/* PACKAGE */
#include <fixposition_driver/converter/imu.hpp>
#include <fixposition_driver/converter/llh.hpp>
#include <fixposition_driver/converter/odometry.hpp>
#include <fixposition_driver/converter/tf.hpp>
#include <fixposition_driver/fixposition_driver.hpp>
#include <fixposition_driver/helper.hpp>

namespace fixposition {
FixpositionDriver::FixpositionDriver(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    // read parameters
    if (!params_.LoadFromRos(node)) {
        ROSFatalError(node_, "Parameter Loading failed, shutting down...");
    }

    Connect();

    ws_sub_ = node_->create_subscription<fixposition_driver::msg::Speed>(params_.customer_input.speed_topic, 100,
                                                       std::bind(&FixpositionDriver::WsCallback, this,
                                                       std::placeholders::_1));

    // static headers
    rawdmi_.head1 = 0xaa;
    rawdmi_.head2 = 0x44;
    rawdmi_.head3 = 0x13;
    rawdmi_.payloadLen = 20;
    rawdmi_.msgId = 2269;
    // these to be filled by each rosmsg
    rawdmi_.wno = 0;
    rawdmi_.tow = 0;
    rawdmi_.dmi1 = 0;
    rawdmi_.dmi2 = 0;
    rawdmi_.dmi3 = 0;
    rawdmi_.dmi4 = 0;
    rawdmi_.mask = 0;

    // initialize converters
    if (!InitializeConverters()) {
        ROSFatalError(node_, "Could not initialize output converter!");
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
            ROSFatalError(node_, "Unknown connection type.");
            return false;
    }
}

void FixpositionDriver::WsCallback(const fixposition_driver::msg::Speed::ConstSharedPtr msg) {
    if (msg->speeds.size() == 1) {
        rawdmi_.dmi1 = msg->speeds[0];
        rawdmi_.mask = (1 << 0) | (0 << 1) | (0 << 2) | (0 << 3);
    } else if (msg->speeds.size() == 2) {
        rawdmi_.dmi1 = msg->speeds[0];
        rawdmi_.dmi2 = msg->speeds[1];
        rawdmi_.mask = (1 << 0) | (1 << 1) | (0 << 2) | (0 << 3) | (1 << 11);    
    } else if (msg->speeds.size() == 4) {
        rawdmi_.dmi1 = msg->speeds[0];
        rawdmi_.dmi2 = msg->speeds[1];
        rawdmi_.dmi3 = msg->speeds[2];
        rawdmi_.dmi4 = msg->speeds[3];
        rawdmi_.mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1, "Invalid speed message with size %lu, the size should be either 1, 2 or 4!",
                          msg->speeds.size());
        return;
    }

    // Calculate CRC
    const uint32_t checksum = crc32((const uint8_t *)&rawdmi_, sizeof(rawdmi_));

    // Compose entire message
    uint8_t message[sizeof(rawdmi_) + sizeof(checksum)];
    memcpy(&message[0], &rawdmi_, sizeof(rawdmi_));
    memcpy(&message[sizeof(rawdmi_)], &checksum, sizeof(checksum));

    send(this->client_fd_, &message[0], sizeof(message), MSG_DONTWAIT);
}

bool FixpositionDriver::InitializeConverters() {
    for (const auto &format : params_.fp_output.formats) {
        if (format == "ODOMETRY") {
            converters_["ODOMETRY"] = std::unique_ptr<OdometryConverter>(new OdometryConverter(node_));
            converters_["TF"] = std::unique_ptr<TfConverter>(new TfConverter(node_));
        } else if (format == "LLH") {
            converters_["LLH"] = std::unique_ptr<LlhConverter>(new LlhConverter(node_));
        } else if (format == "RAWIMU") {
            converters_["RAWIMU"] = std::unique_ptr<ImuConverter>(new ImuConverter(node_, false));
        } else if (format == "CORRIMU") {
            converters_["CORRIMU"] = std::unique_ptr<ImuConverter>(new ImuConverter(node_, true));
        } else if (format == "TF") {
            if (converters_.find("TF") == converters_.end()) {
                converters_["TF"] = std::unique_ptr<TfConverter>(new TfConverter(node_));
            }
        } else {
            RCLCPP_INFO_STREAM(node_->get_logger(), "Unknown input format: " << format);
        }
    }
    return !converters_.empty();
}

void FixpositionDriver::Run() {
    rclcpp::Rate rate(params_.fp_output.rate);
    while (rclcpp::ok()) {
        if ((client_fd_ > 0) && (connection_status_ == 0) && ReadAndPublish()) {
            rclcpp::spin_some(node_);
            rate.sleep();
        } else {
            RCLCPP_INFO(node_->get_logger(), "Reconnecting in %.1f seconds ...", params_.fp_output.reconnect_delay);
            close(client_fd_);
            client_fd_ = -1;

            rclcpp::spin_some(node_);
	    std::chrono::nanoseconds reconnect_delay = std::chrono::nanoseconds((uint64_t)params_.fp_output.reconnect_delay * 1000 * 1000 * 1000);
            rclcpp::sleep_for(reconnect_delay);

            Connect();
        }
    }
}

bool FixpositionDriver::ReadAndPublish() {
    char readBuf[8192];

    ssize_t rv;
    if (params_.fp_output.type == INPUT_TYPE::TCP) {
        rv = recv(client_fd_, (void *)&readBuf, sizeof(readBuf), MSG_DONTWAIT);
    } else if (params_.fp_output.type == INPUT_TYPE::SERIAL) {
        rv = read(client_fd_, (void *)&readBuf, sizeof(readBuf));
    } else {
        rv = 0;
    }

    if (rv == 0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Connection closed.");
        return false;
    }

    if (rv < 0 && errno == EAGAIN) {
        /* no data for now, call back when the socket is readable */
        return true;
    }
    if (rv < 0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Connection error.");
        return false;
    }

    // find start of a NMEA style message
    char *start = (char *)memchr(readBuf, '$', rv);
    while (start != NULL) {
        // check if it is NMEA with correct checksum
        auto nmea_size = IsNmeaMessage(start, rv - (start - readBuf));

        // convert the message into ros messages
        if (nmea_size > 0) {
            std::string msg(start, nmea_size);
            ConvertAndPublish(msg);

            // move to next $ for next message
            start = (char *)memchr(start + 1, '$', rv);
        } else {
            break;
        }
    }

    return true;
}

void FixpositionDriver::ConvertAndPublish(const std::string &msg) {
    // split the msg into tokens, removing the *XX checksum
    std::vector<std::string> tokens;
    std::size_t star_pos = msg.find_last_of("*");
    SplitMessage(tokens, msg.substr(1, star_pos - 1), ",");

    // if it doesn't start with FP then do nothing
    if (tokens.at(0) != "FP") {
        return;
    }

    // Get the header of the sentence
    const std::string header = tokens.at(1);


    // If we have a converter available, convert to ros. Currently supported are "FP" and "LLH"
    if (converters_[header] != nullptr) {
        converters_[header]->ConvertTokensAndPublish(tokens);
    }
}

bool FixpositionDriver::CreateTCPSocket() {
    struct sockaddr_in server_address;
    client_fd_ = socket(AF_INET, SOCK_STREAM, 0);

    if (client_fd_ < 0) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 5, "Error in client creation.");
        return false;
    } else {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Client created.");
    }

    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(std::stoi(params_.fp_output.port));
    server_address.sin_addr.s_addr = inet_addr(params_.fp_output.ip.c_str());

    connection_status_ = connect(client_fd_, (struct sockaddr *)&server_address, sizeof server_address);

    if (connection_status_ != 0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error on connection of TCP socket: " << strerror(errno));
        return false;
    }
    return true;
}

bool FixpositionDriver::CreateSerialConnection() {
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
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Unsupported baudrate: " << params_.fp_output.baudrate
                                                      << "\n\tsupported examples:\n\t9600, "
                                                         "19200, "
                                                         "38400, "
                                                         "57600\t\n115200\n230400\n460800\n500000\n921600\n1000000");
    }

    if (client_fd_ == -1) {
        // Could not open the port.
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 5, "Failed to open serial port " << strerror(errno));
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
