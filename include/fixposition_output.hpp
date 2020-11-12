/**
 * @file fixposition_output.hpp
 * @author Andreea Lutac (andreea.lutac@fixposition.ch)
 * @brief
 * version 0.1
 * @date 2020-11-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef FIXPOSITION_OUTPUT
#define FIXPOSITION_OUTPUT

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <nav_msgs/Odometry.h>
#include <net/if.h>  //ifreq
#include <netdb.h>
#include <netinet/in.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "base_converter.hpp"
#include "fp_msg_converter.hpp"

enum INPUT_TYPE { tcp = 1, serial = 2 };

class FixpositionOutput {
   public:
    FixpositionOutput(ros::NodeHandle* nh, const int rate);
    ~FixpositionOutput();
    bool InitializeInputConverter();
    bool TCPReadAndPublish();
    bool SerialReadAndPublish();
    bool CreateTCPSocket(const int port, const std::string& ip);
    bool CreateSerialConnection(const char* name, int baudrate = 115200, int read_timeout_s = 5);
    void Run();
    static void ROSFatalError(const std::string& error);

   private:
    ros::NodeHandle nh_;
    int rate_;
    INPUT_TYPE input_type_;
    std::string input_format_;
    std::string tcp_ip_;
    std::string input_port_;
    int serial_baudrate_;
    int client_fd_ = -1;  //!< TCP/IP socket
    int serial_fd_ = -1;  //!< Serial file descriptor
    struct termios options_save_;
    char inbuf_[113];
    size_t inbuf_used_ = 0;

    ros::Publisher odometry_pub_;

    std::unique_ptr<BaseConverter> converter_;
};

#endif