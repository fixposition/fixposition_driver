/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /
 *   /  /\  \
 *  /__/  \__\  Fixposition AG
 *
 * @file fixposition_driver.cpp
 * @author Kailin Huang (kailin.huang@fixposition.com)
 * @brief
 * @date 2022-01-26
 *
 */

/* SYSTEM / STL */

/* EXTERNAL */

/* ROS */

/* PACKAGE */
#include <fixposition_driver/converter/fp_converter.hpp>
#include <fixposition_driver/converter/llh_converter.hpp>
#include <fixposition_driver/fixposition_driver.hpp>
#include <fixposition_driver/helper.hpp>

namespace fixposition {
FixpositionDriver::FixpositionDriver(ros::NodeHandle *nh) : nh_(*nh) {
    // read parameters
    if (!ros::param::get("~/rate", rate_)) rate_ = 100;

    std::string type;
    if (!ros::param::get("~/input_type", type)) {
        ROSFatalError("Missing parameter: input_type");
    }
    if (type == "tcp") {
        input_type_ = INPUT_TYPE::TCP;
    } else if (type == "serial") {
        input_type_ = INPUT_TYPE::SERIAL;
    } else {
        ROSFatalError("Unknown input type! Must be either tcp or serial.");
    }

    if (!ros::param::get("~/input_formats", input_formats_)) input_formats_ = {"FP", "LLH"};

    // Get parameters: port (required)
    if (!ros::param::get("~/input_port", input_port_)) {
        ROSFatalError("Missing parameter: input_port");
    }
    if (input_type_ == INPUT_TYPE::TCP) {
        if (!ros::param::get("~/tcp_ip", tcp_ip_)) tcp_ip_ = "10.0.1.1";
        CreateTCPSocket();
        if (client_fd_ <= 0) {
            ROSFatalError("Could not open socket.");
        }
    } else if (input_type_ == INPUT_TYPE::SERIAL) {
        if (!ros::param::get("~/serial_baudrate", serial_baudrate_)) serial_baudrate_ = 115200;
        CreateSerialConnection();
        if (client_fd_ <= 0) {
            ROSFatalError("Could not configure serial port.");
        }
    } else {
        ROSFatalError("Unknown output type.");
    }

    // initialize converters
    if (!InitializeConverters()) {
        ROSFatalError("Could not initialize output converter!");
    }
}

FixpositionDriver::~FixpositionDriver() {
    if (client_fd_ != -1) {
        if (input_type_ == INPUT_TYPE::SERIAL) {
            tcsetattr(client_fd_, TCSANOW, &options_save_);
        }
        close(client_fd_);
    }
}

void FixpositionDriver::ROSFatalError(const std::string &error) {
    ROS_ERROR_STREAM(error);
    ros::shutdown();
}

bool FixpositionDriver::InitializeConverters() {
    for (const auto format : input_formats_) {
        if (format == "FP") {
            converters_["FP"] = std::unique_ptr<FpConverter>(new FpConverter(nh_));
        } else if (format == "LLH") {
            converters_["LLH"] = std::unique_ptr<LlhConverter>(new LlhConverter(nh_));
        } else {
            ROS_ERROR_STREAM("Unknown input format: " << format);
        }
    }
    return !converters_.empty();
}
void FixpositionDriver::Run() {
    ros::Rate rate(rate_);
    int res_counter = 0;
    while (ros::ok()) {
        if (client_fd_ > 0) {
            bool ret = ReadAndPublish();
        }
        ros::spinOnce();
        rate.sleep();
    }
}

bool FixpositionDriver::ReadAndPublish() {
    char readBuf[8192];
    ssize_t rv;
    if (input_type_ == INPUT_TYPE::TCP) {
        rv = recv(client_fd_, (void *)&readBuf, sizeof(readBuf), MSG_DONTWAIT);
    } else if (input_type_ == INPUT_TYPE::SERIAL) {
        rv = read(client_fd_, (void *)&readBuf, sizeof(readBuf));
    }
    if (rv == 0) {
        ROS_ERROR_STREAM("Connection closed.");
        return false;
    }
    if (rv < 0 && errno == EAGAIN) {
        /* no data for now, call back when the socket is readable */
        return false;
    }
    if (rv < 0) {
        ROS_ERROR_STREAM("Connection error.");
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
    split_message(tokens, msg.substr(1, star_pos - 1), ",");

    // Get the header of the sentence
    const std::string header = tokens.at(0);

    // If we have a converter available, convert to ros. Currently supported are "FP" and "LLH"
    if (converters_[header] != nullptr) {
        converters_[header]->ConvertTokensAndPublish(tokens);
    }
}

bool FixpositionDriver::CreateTCPSocket() {
    struct sockaddr_in server_address;
    client_fd_ = socket(AF_INET, SOCK_STREAM, 0);

    if (client_fd_ < 0) {
        ROS_ERROR_STREAM("Error in client creation.");
        return false;
    } else
        ROS_INFO_STREAM("Client created.");

    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(std::stoi(input_port_));
    server_address.sin_addr.s_addr = inet_addr(tcp_ip_.c_str());

    int connection_status = connect(client_fd_, (struct sockaddr *)&server_address, sizeof server_address);

    if (connection_status != 0) {
        ROS_ERROR_STREAM("Error on connection.");
        return false;
    }
    return true;
}

bool FixpositionDriver::CreateSerialConnection() {
    client_fd_ = open(input_port_.c_str(), O_RDWR | O_NOCTTY);

    struct termios options;
    speed_t speed;

    switch (serial_baudrate_) {
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
            ROS_ERROR_STREAM("Unsupported baudrate: " << serial_baudrate_
                                                      << "\n\tsupported examples:\n\t9600, "
                                                         "19200, "
                                                         "38400, "
                                                         "57600\t\n115200\n230400\n460800\n500000\n921600\n1000000");
    }

    if (client_fd_ == -1) {
        // Could not open the port.
        std::cerr << "Failed to open serial port " << strerror(errno) << std::endl;
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
    }
    return true;
}
}  // namespace fixposition