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

FixpositionOutput::FixpositionOutput(ros::NodeHandle *nh, const int rate) : nh_(*nh), rate_(rate) {
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
        CreateTCPSocket(std::stoi(input_port_), tcp_ip_);
        if (client_fd_ <= 0) {
            ROSFatalError("Could not open socket.");
        }
    } else if (input_type_ == INPUT_TYPE::serial) {
        if (!ros::param::get("~/serial_baudrate", serial_baudrate_)) serial_baudrate_ = 115200;
        CreateSerialConnection(input_port_.c_str(), serial_baudrate_);
        if (serial_fd_ <= 0) {
            ROSFatalError("Could not configure serial port.");
        }
    } else {
        ROSFatalError("Unknown output type.");
    }

    if (!InitializeInputConverter()) {
        ROSFatalError("Could not initialize output converter!");
    }
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry", 100);
}

FixpositionOutput::~FixpositionOutput() {
    if (input_type_ == INPUT_TYPE::tcp) {
        if (client_fd_ != -1) {
            // Reset settings:
            tcsetattr(client_fd_, TCSANOW, &options_save_);
            close(client_fd_);
        }
    } else if (input_type_ == INPUT_TYPE::serial) {
        if (serial_fd_ != -1) {
            close(serial_fd_);
        }
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
    bool ret;
    while (ros::ok()) {
        if (input_type_ == INPUT_TYPE::tcp) {
            ret = TCPReadAndPublish();
        } else if (input_type_ == INPUT_TYPE::serial) {
            ret = SerialReadAndPublish();
        }

        if (!ret) {
            if (res_counter > 10) {
                ROS_FATAL("Too many connection or publishing failures. Exiting...");
                ros::shutdown();
            }
            res_counter++;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

bool FixpositionOutput::TCPReadAndPublish() {
    size_t inbuf_remain = sizeof(inbuf_) - inbuf_used_;
    if (inbuf_remain == 0) {
        fprintf(stderr, "Line exceeded buffer length!\n");
        abort();
    }

    ssize_t rv = recv(client_fd_, (void *)&inbuf_[inbuf_used_], inbuf_remain, MSG_DONTWAIT);
    if (rv == 0) {
        fprintf(stderr, "Connection closed.\n");
        abort();
    }
    if (rv < 0 && errno == EAGAIN) {
        /* no data for now, call back when the socket is readable */
        return false;
    }
    if (rv < 0) {
        perror("Connection error");
        abort();
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
        nav_msgs::Odometry msg = converter_->convert(str_state);
        odometry_pub_.publish(msg);
        line_start = line_end + 1;
    }
    /* Shift buffer down so the unprocessed data is at the start */
    inbuf_used_ -= (line_start - inbuf_);
    memmove(inbuf_, line_start, inbuf_used_);
    return true;
}

bool FixpositionOutput::SerialReadAndPublish() {
    if (serial_fd_ >= 0) {
        ssize_t rd = read(serial_fd_, inbuf_, 113);
        if (rd >= 0) {
            ROS_DEBUG_STREAM("Read " << rd << " bytes from fd " << serial_fd_);
            inbuf_[rd] = '\0';
            std::string str_state = inbuf_;
            nav_msgs::Odometry msg = converter_->convert(str_state);
            odometry_pub_.publish(msg);
            return true;
        } else {
            ROS_ERROR_STREAM("No data read from fd " << serial_fd_);
            return false;
        }
    } else {
        ROS_FATAL("Serial file descriptor is not open.");
        return false;
    }
}

bool FixpositionOutput::CreateTCPSocket(const int port, const std::string &ip) {
    const char *ip_address;
    int connection_status;
    struct sockaddr_in server_address;
    client_fd_ = socket(AF_INET, SOCK_STREAM, 0);

    if (client_fd_ < 0) {
        std::cerr << "Error in client creation." << std::endl;
        return false;
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

bool FixpositionOutput::CreateSerialConnection(const char *name, int baudrate = 115200, int read_timeout_s = 5) {
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
        return false;
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
    return true;
}