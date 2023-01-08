/**
 *  @file
 *  @brief Parameters
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_PARAMS_HPP__

/* SYSTEM / STL */
#include <string>
#include <vector>

/* ROS */
#include <rclcpp/rclcpp.hpp>

/* EXTERNAL */

/* PACKAGE */

namespace fixposition {

enum class INPUT_TYPE { TCP = 1, SERIAL = 2 };

struct FpOutputParams {
    std::shared_ptr<rclcpp::Node> node_;
    int rate;                          //!< loop rate of the main read loop
    double reconnect_delay;            //!< wait time in [s] until retry connection
    INPUT_TYPE type;                   //!< TCP or SERIAL
    std::vector<std::string> formats;  //!< data formats to convert, support "FP" and "LLH" for now

    std::string ip;    //!< IP address for TCP connection
    std::string port;  //!< Port for TCP connection
    int baudrate;      //!< baudrate of serial connection

    explicit FpOutputParams(std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief Load all parameters from ROS parameter server
     *
     * @return true success
     * @return false fail
     */
    bool LoadFromRos();
};

struct CustomerInputParams {
    std::shared_ptr<rclcpp::Node> node_;
    std::string speed_topic;
    explicit CustomerInputParams(std::shared_ptr<rclcpp::Node> node);
    /**
     * @brief Load all parameters from ROS parameter server
     *
     * @param[in] ns namespace to load the parameters from
     * @return true success
     * @return false fail
     */
    bool LoadFromRos();
};

struct FixpositionDriverParams {
    std::unique_ptr<FpOutputParams> fp_output;
    std::unique_ptr<CustomerInputParams> customer_input;
    bool LoadFromRos(std::shared_ptr<rclcpp::Node> node);
};

}  // namespace fixposition

#endif
