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

FpOutputParams::FpOutputParams(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
    node_->declare_parameter(RATE, 100);
    node_->declare_parameter(RECONNECT_DELAY, 5.0);
    node_->declare_parameter(TYPE, "");
    node_->declare_parameter(FORMATS, std::vector<std::string>());
    node_->declare_parameter(PORT, "");
    node_->declare_parameter(IP, "127.0.0.1");
    node_->declare_parameter(BAUDRATE, 115200);
} 

bool FpOutputParams::LoadFromRos() {
    // read parameters
    node_->get_parameter(RATE, rate);
    RCLCPP_INFO(node_->get_logger(), "%s : %d", RATE.c_str(), rate);
    node_->get_parameter(RECONNECT_DELAY, reconnect_delay);
    RCLCPP_INFO(node_->get_logger(), "%s : %f", RECONNECT_DELAY.c_str(), reconnect_delay);

    std::string type_str;
    node_->get_parameter(TYPE, type_str);
    if (type_str == "tcp") {
        type = INPUT_TYPE::TCP;
    } else if (type_str == "serial") {
        type = INPUT_TYPE::SERIAL;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Input type has to be tcp or serial!");
        return false;
    }

    node_->get_parameter(FORMATS, formats);
    for (size_t i = 0; i < formats.size(); i++)
    	RCLCPP_INFO(node_->get_logger(), "%s[%ld] : %s", FORMATS.c_str(), i, formats[i].c_str());

    // Get parameters: port (required)
    node_->get_parameter(PORT, port);
    RCLCPP_INFO(node_->get_logger(), "%s : %s", PORT.c_str(), port.c_str());

    if (type == INPUT_TYPE::TCP) {
        node_->get_parameter(IP, ip);
        RCLCPP_INFO(node_->get_logger(), "%s : %s", IP.c_str(), ip.c_str());
    } else if (type == INPUT_TYPE::SERIAL) {
        node_->get_parameter(BAUDRATE, baudrate);
    RCLCPP_INFO(node_->get_logger(), "%s : %d", BAUDRATE.c_str(), baudrate);
    }

    return true;
}

CustomerInputParams::CustomerInputParams(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
    node_->declare_parameter(SPEED_TOPIC, "/fixposition/speed");
}

bool CustomerInputParams::LoadFromRos() {
    node_->get_parameter(SPEED_TOPIC, speed_topic);
    RCLCPP_INFO(node_->get_logger(), "%s : %s", SPEED_TOPIC.c_str(), speed_topic.c_str());
    return true;
}

bool FixpositionDriverParams::LoadFromRos(std::shared_ptr<rclcpp::Node> node) {
    bool ok = true;
    fp_output = std::make_unique<FpOutputParams>(node);
    customer_input = std::make_unique<CustomerInputParams>(node);

    ok &= fp_output->LoadFromRos();
    ok &= customer_input->LoadFromRos();

    return ok;
}

}  // namespace fixposition
