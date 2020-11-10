/**
 * @file fixposition_output.cpp
 * @author Andreea Lutac (andreea.lutac@fixposition.ch)
 * @brief
 * version 0.1
 * @date 2020-11-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "fixposition_output.hpp"

FixpositionOutput::FixpositionOutput(ros::NodeHandle *nh, const OUTPUT_TYPE &type) : nh_(*nh), type_(type) {
    nh_.param<std::string>("input_format", input_format_, "fp");

    // Get parameters: port (required)
    nh_.param<std::string>("tcp_ip", tcp_address_, "192.168.49.1");
    if (!nh_.getParam("input_port", input_port_){
        throw "Missing parameter: input_port";
    }
    if(type_==OUTPUT_TYPE::tcp){
        CreateTCPSocket(input_port_, tcp_address_);
        if (client_fd_ <= 0) {
            throw "Could not open socket.";
        }
    } else if(type_==OUTPUT_TYPE::serial){
        nh_.param<int>("serial_baudrate", serial_baudrate_, 115200);
        CreateSerialConnection(input_port_.c_str(), serial_baudrate_);
        if (serial_fd_ <= 0) {
            throw "Could not configure serial port.";
        }
    }
     if (!InitializeInputConverter()) {
        throw "Could not initialize output converter!";
    }
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry", 100);
}

bool FixpositionOutput::InitializeInputConverter() {
    switch (input_format_) {
        case OUTPUT_FORMAT::FP:
            converter_ = std::unique_ptr<FpMsgConverter>(new FpMsgConverter());
            break;
        default:
            ROS_ERROR_STREAM("Unknown input format: " << input_format_);
            break;
    }
    converter_->LoadParameters();
    return true;
}
void FixpositionOutput::Run() {}
void FixpositionOutput::CreateTCPSocket(const int port, const std::string &ip) {
    const char *ip_address;
    int connection_status;
    struct sockaddr_in server_address;
    client_fd_ = socket(AF_INET, SOCK_STREAM, 0);

    if (client_fd_ < 0) {
        std::cerr << "Error in client creation." << std::endl;
        return -1;
    } else
        std::cout << "Client created." << std::endl;

    ip_address = ip.c_str();

    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(port);
    server_address.sin_addr.s_addr = inet_addr(ip_address);

    connection_status = connect(client_fd_, (struct sockaddr *)&server_address, sizeof server_address);

    if (connection_status != 0) return false;
    return true;
}

void FixpositionOutput::CreateSerialConnection(const char *name, int baudrate = 115200, int read_timeout_s = 5) {
    serial_fd_ = open(name, O_RDWR | O_NOCTTY);

    struct termios options;
    speed_t speed;

    switch (baudrate) {
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
            ROS_ERROR_STREAM("Unsupported baudrate: " << baudrate
                                                      << "\n\tsupported examples:\n\t9600, "
                                                         "19200, "
                                                         "38400, "
                                                         "57600\t\n115200\n230400\n460800\n500000\n921600\n1000000");
    }

    if (serial_fd_ == -1) {
        // Could not open the port.
        std::cerr << "Failed to open serial port " << strerror(errno) << std::endl;
    } else {
        // Get current serial port options:
        tcgetattr(serial_fd_, &options);
        options_save_ = options;
        char speed_buf[10];
        snprintf(speed_buf, sizeof(speed_buf), "0%06o", (int)cfgetispeed(&options));

        options.c_iflag &= ~(IXOFF | IXON | ICRNL);
        options.c_oflag &= ~(OPOST | ONLCR);
        options.c_lflag &= ~(ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN);
        options.c_cc[VEOL] = 0;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = read_timeout_s * 10;

        cfsetospeed(&options, speed); /* baud rate */
        tcsetattr(serial_fd_, TCSANOW, &options);
    }
}