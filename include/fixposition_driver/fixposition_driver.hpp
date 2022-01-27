/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /
 *   /  /\  \
 *  /__/  \__\  Fixposition AG
 *
 * @file fixposition_driver.hpp
 * @author Kailin Huang (kailin.huang@fixposition.com)
 * @brief
 * @date 2022-01-26
 *
 */
#ifndef __FIXPOSITION_DRIVER_FIXPOSITION_DRIVER_HPP__
#define __FIXPOSITION_DRIVER_FIXPOSITION_DRIVER_HPP__

/* SYSTEM / STL */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <net/if.h>  //ifreq
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <unordered_map>
/* EXTERNAL */

/* ROS */
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

/* PACKAGE */
#include <fixposition_driver/VRTK.h>

#include <fixposition_driver/converter/base_converter.hpp>

namespace fixposition {

enum class INPUT_TYPE { TCP = 1, SERIAL = 2 };

class FixpositionDriver {
   public:
    /**
     * @brief Construct a new FixpositionDriver object
     *
     * @param[in] nh node handle
     */
    FixpositionDriver(ros::NodeHandle* nh);

    /**
     * @brief Destroy the Fixposition Driver object, close all open connections
     *
     */
    ~FixpositionDriver();

    /**
     * @brief Run in Loop the Read Convert and Publish cycle
     *
     */
    void Run();

    /**
     * @brief Send ROS Fatal error and exit
     *
     * @param[in] error Error msg to be sent
     */
    static void ROSFatalError(const std::string& error);

   private:
    /**
     * @brief Convert the string using correct converter
     *
     * @param[in] msg NMEA like string to be converted. $HEADER,,,,,,,*CHECKSUM
     */
    void ConvertAndPublish(const std::string& msg);

    /**
     * @brief Initialize convertes based on config
     *
     * @return true
     * @return false
     */
    bool InitializeConverters();

    /**
     * @brief Read data and publish to ros if possible
     *
     * @return true data read success
     * @return false no data read or connection problems
     */
    bool ReadAndPublish();

    /**
     * @brief Initialize TCP connection
     *
     * @return true success
     * @return false fail
     */
    bool CreateTCPSocket();

    /**
     * @brief Initialize Serial connection
     *
     * @return true success
     * @return false fail
     */
    bool CreateSerialConnection();

    ros::NodeHandle nh_;
    int rate_;                                //!< loop rate of the main read loop
    INPUT_TYPE input_type_;                   //!< TCP or SERIAL
    std::vector<std::string> input_formats_;  //!< data formats to convert, support "FP" and "LLH" for now

    std::unordered_map<std::string, std::unique_ptr<BaseConverter>>
        converters_;          //!< converters corresponding to the input formats
    std::string tcp_ip_;      //!< IP address for TCP connection
    std::string input_port_;  //! Port for TCP connection
    int serial_baudrate_;     //!< baudrate of serial connection
    int client_fd_ = -1;      //!< TCP or Serial file descriptor
    struct termios options_save_;
};
}  // namespace fixposition
#endif