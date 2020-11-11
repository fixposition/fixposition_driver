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
    FixpositionOutput(ros::NodeHandle* nh, const INPUT_TYPE& type);
    ~FixpositionOutput();
    bool InitializeInputConverter();
    void CreateTCPSocket(const int port, const std::string& ip);
    void CreateSerialConnection(const char* name, int baudrate = 115200, int read_timeout_s = 5);
    void Run();

   private:
    ros::NodeHandle nh_;
    INPUT_TYPE type;
    std::string tcp_ip_;
    int input_port_;
    int client_fd_ = -1;  //!< TCP/IP socket
    int serial_fd_ = -1;  //!< Serial file descriptor
    struct termios options_save_;
    char inbuf_[113];
    size_t inbuf_used_ = 0;

    ros::Publisher odometry_pub_;

    std::unique_ptr<BaseConverter> converter_;
}