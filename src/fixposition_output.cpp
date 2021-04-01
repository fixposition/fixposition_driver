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

FixpositionOutput::FixpositionOutput(ros::NodeHandle *nh) : nh_(*nh) {
    if (!ros::param::get("~/pub_rate", rate_)) rate_ = 200;

    std::string type;
    if (!ros::param::get("~/input_type", type)) {
        ROSFatalError("Missing parameter: input_type");
    }
    if (type == "tcp") {
        input_type_ = INPUT_TYPE::tcp;
    } else if (type == "serial") {
        input_type_ = INPUT_TYPE::serial;
    } else {
        ROSFatalError("Unknown input type! Must be either tcp or serial.");
    }

    if (!ros::param::get("~/input_format", input_format_)) input_format_ = "fp";

    // Get parameters: port (required)
    if (!ros::param::get("~/input_port", input_port_)) {
        ROSFatalError("Missing parameter: input_port");
    }
    if (input_type_ == INPUT_TYPE::tcp) {
        if (!ros::param::get("~/tcp_ip", tcp_ip_)) tcp_ip_ = "192.168.49.1";
        CreateTCPSocket();
        if (client_fd_ <= 0) {
            ROSFatalError("Could not open socket.");
        }
    } else if (input_type_ == INPUT_TYPE::serial) {
        if (!ros::param::get("~/serial_baudrate", serial_baudrate_)) serial_baudrate_ = 115200;
        CreateSerialConnection();
        if (client_fd_ <= 0) {
            ROSFatalError("Could not configure serial port.");
        }
    } else {
        ROSFatalError("Unknown output type.");
    }

    if (!InitializeInputConverter()) {
        ROSFatalError("Could not initialize output converter!");
    }
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry", 100);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/fixposition/imu", 100);
    navsat_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/navsatfix", 100);
    status_pub_ = nh_.advertise<fixposition_output::VRTK>("/fixposition/vrtk", 100);
}

FixpositionOutput::~FixpositionOutput() {
    if (client_fd_ != -1) {
        if (input_type_ == INPUT_TYPE::serial) {
            tcsetattr(client_fd_, TCSANOW, &options_save_);
        }
        close(client_fd_);
    }
}

void FixpositionOutput::ROSFatalError(const std::string &error) {
    ROS_ERROR_STREAM(error);
    ros::shutdown();
}

bool FixpositionOutput::InitializeInputConverter() {
    if (input_format_ == "fp") {
        converter_ = std::unique_ptr<FpMsgConverter>(new FpMsgConverter());
    } else {
        ROS_ERROR_STREAM("Unknown input format: " << input_format_);
        return false;
    }
    return true;
}
void FixpositionOutput::Run() {
    ros::Rate rate(rate_);
    int res_counter = 0;
    while (ros::ok()) {
        if (client_fd_ > 0) {
            bool ret = ReadAndPublish();
            // if (!ret) {
            //     if (res_counter > 10) {
            //         ROS_FATAL("Too many connection or publishing failures. Exiting...");
            //         ros::shutdown();
            //     }
            //     res_counter++;
            // }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

bool FixpositionOutput::ReadAndPublish() {
    size_t inbuf_remain = sizeof(inbuf_) - inbuf_used_;
    if (inbuf_remain == 0) {
        ROS_ERROR_STREAM("Line exceeded buffer length!");
        return false;
    }
    ssize_t rv;
    if (input_type_ == INPUT_TYPE::tcp) {
        rv = recv(client_fd_, (void *)&inbuf_[inbuf_used_], inbuf_remain, MSG_DONTWAIT);

    } else if (input_type_ == INPUT_TYPE::serial) {
        rv = read(client_fd_, (void *)&inbuf_[inbuf_used_], inbuf_remain);
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
    inbuf_used_ += rv;

    /* Scan for newlines in the line buffer; we're careful here to deal with embedded \0s
     * an evil server may send, as well as only processing lines that are complete.
     */
    char *line_start = inbuf_;
    char *line_end;
    while ((line_end = (char *)memchr((void *)line_start, '\n', inbuf_used_ - (line_start - inbuf_)))) {
        *line_end = 0;
        std::string str_state = line_start;
        converter_->convertAndPublish(str_state, odometry_pub_, imu_pub_, navsat_pub_, status_pub_);
        line_start = line_end + 1;
    }
    /* Shift buffer down so the unprocessed data is at the start */
    inbuf_used_ -= (line_start - inbuf_);
    memmove(inbuf_, line_start, inbuf_used_);
    return true;
}

bool FixpositionOutput::CreateTCPSocket() {
    const char *ip_address;
    int connection_status;
    struct sockaddr_in server_address;
    client_fd_ = socket(AF_INET, SOCK_STREAM, 0);

    if (client_fd_ < 0) {
        ROS_ERROR_STREAM("Error in client creation.");
        return false;
    } else
        ROS_INFO_STREAM("Client created.");

    ip_address = tcp_ip_.c_str();

    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(std::stoi(input_port_));
    server_address.sin_addr.s_addr = inet_addr(ip_address);

    connection_status = connect(client_fd_, (struct sockaddr *)&server_address, sizeof server_address);

    if (connection_status != 0) {
        ROS_ERROR_STREAM("Error on connection.");
        return false;
    }
    return true;
}

bool FixpositionOutput::CreateSerialConnection() {
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