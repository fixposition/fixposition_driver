/**
 *  @file
 *  @brief Declaration of FixpositionDriver class
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_FIXPOSITION_DRIVER__
#define __FIXPOSITION_DRIVER_FIXPOSITION_DRIVER__

/* SYSTEM / STL */
#include <termios.h>

#include <unordered_map>

/* EXTERNAL */

/* ROS */
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

/* PACKAGE */
#include <fixposition_driver/Speed.h>
#include <fixposition_driver/VRTK.h>

#include <fixposition_driver/converter/base_converter.hpp>
#include <fixposition_driver/params.hpp>
#include <fixposition_driver/rawdmi.hpp>

namespace fixposition {

class FixpositionDriver {
   public:
    /**
     * @brief Construct a new FixpositionDriver object
     *
     * @param[in] nh node handle
     */
    FixpositionDriver(ros::NodeHandle *nh);

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

   private:
    /**
     * @brief
     *
     * @param[in] msg
     */
    void WsCallback(fixposition_driver::Speed msg);

    /**
     * @brief Convert the string using correct converter
     *
     * @param[in] msg NMEA like string to be converted. $HEADER,,,,,,,*CHECKSUM
     */
    void ConvertAndPublish(const std::string &msg);

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
     * @return true data read success or no data
     * @return false connection problems, restart the connection
     */
    bool ReadAndPublish();

    /**
     * @brief Connect the defined TCP or Serial socket
     *
     * @return true success
     * @return false cannot connect
     */
    bool Connect();

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
    ros::Subscriber ws_sub_;  //!< wheelspeed message subscriber

    RAWDMI rawdmi_;  //!< RAWDMI msg struct

    FixpositionDriverParams params_;

    std::unordered_map<std::string, std::unique_ptr<BaseConverter>>
        converters_;  //!< converters corresponding to the input formats

    int client_fd_ = -1;          //!< TCP or Serial file descriptor
    int connection_status_ = -1;  //!<
    struct termios options_save_;
};
}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_FIXPOSITION_DRIVER__
