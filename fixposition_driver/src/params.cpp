/**
 *  @file
 *  @brief Implementation of Parameter Loading
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* ROS */
#include <rclcpp/rclcpp.hpp>

/* PACKAGE */
#include <fixposition_driver/params.hpp>

namespace fixposition {

bool FpOutputParams::LoadFromRos(std::shared_ptr<rclcpp::Node> node, const std::string &ns) {
    // read parameters
    //if (!rclcpp::param::get(ns + "/rate", rate)) 
    rate = 100;

    //if (!rclcpp::param::get(ns + "/reconnect_delay", reconnect_delay))
    reconnect_delay = 5.0;

    std::string type_str;
    //if (!rclcpp::param::get(ns + "/type", type_str)) {
    //    return false;
    //}
    type_str = "tcp";
    if (type_str == "tcp") {
        type = INPUT_TYPE::TCP;
    } else if (type_str == "serial") {
        type = INPUT_TYPE::SERIAL;
    } else {
        //ROS_ERROR("Input type has to be tcp or serial!");
        return false;
    }

    //if (!rclcpp::param::get(ns + "/formats", formats))
      formats = {"ODOMETRY", "LLH", "RAWIMU", "CORRIMU", "TF"};

    // Get parameters: port (required)
    //if (!rclcpp::param::get(ns + "/port", port)) {
    //}

    if (type == INPUT_TYPE::TCP) {
        //if (!rclcpp::param::get(ns + "/ip", ip)) {
            // default value for IP
            ip = "127.0.0.1";
        //}
        //if (!rclcpp::param::get(ns + "/port", port)) {
            // default value for TCP port
            port = "21000";
        //}
    } else if (type == INPUT_TYPE::SERIAL) {
        //if (!rclcpp::param::get(ns + "/baudrate", baudrate)) {
            // default value for baudrate
            baudrate = 115200;
        //}
        //if (!rclcpp::param::get(ns + "/port", port)) {
            // default value for serial port
            port = "/dev/ttyUSB0";
        //}
    }

    return true;
}

bool CustomerInputParams::LoadFromRos(std::shared_ptr<rclcpp::Node> node, const std::string &ns) {
    //if (!rclcpp::param::get(ns + "/speed_topic", speed_topic)) {
        // default value for the topic name
        speed_topic = "/fixposition/speed";
    //}

    return true;
}

bool FixpositionDriverParams::LoadFromRos(std::shared_ptr<rclcpp::Node> node, const std::string &ns) {
    bool ok = true;

    ok &= fp_output.LoadFromRos(node, ns + "fp_output");
    ok &= customer_input.LoadFromRos(node, ns + "customer_input");

    return ok;
}

}  // namespace fixposition
