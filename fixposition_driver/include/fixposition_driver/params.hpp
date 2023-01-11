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

    const std::string RATE = "fp_output.rate";
    int rate;                          //!< loop rate of the main read loop
    const std::string RECONNECT_DELAY = "fp_output.reconnect_delay";
    double reconnect_delay;            //!< wait time in [s] until retry connection
    const std::string TYPE = "fp_output.type";
    INPUT_TYPE type;                   //!< TCP or SERIAL
    const std::string FORMATS = "fp_output.formats";
    std::vector<std::string> formats;  //!< data formats to convert, support "FP" and "LLH" for now
    const std::string IP = "fp_output.ip";
    std::string ip;    //!< IP address for TCP connection
    const std::string PORT = "fp_output.port";
    std::string port;  //!< Port for TCP connection
    const std::string BAUDRATE = "fp_output.baudrate";
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

    const std::string SPEED_TOPIC = "customer_input.speed_topic";
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
