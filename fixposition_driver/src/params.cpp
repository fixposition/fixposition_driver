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
    node_->declare_parameter("fp_output.rate", 100);
    node_->declare_parameter("fp_output.reconnect_delay", 5.0);
    node_->declare_parameter("fp_output.type", "");
    node_->declare_parameter("fp_output.formats", std::vector<std::string>());
    node_->declare_parameter("fp_output.port", "");
    node_->declare_parameter("fp_output.ip", "127.0.0.1");
    node_->declare_parameter("fp_output.baudrate", 115200);
} 

bool FpOutputParams::LoadFromRos() {
    // read parameters
    node_->get_parameter("fp_output.rate", rate);
    RCLCPP_INFO(node_->get_logger(), "fp_output.rate : %d", rate);
    node_->get_parameter("fp_output.reconnect_delay", reconnect_delay);
    RCLCPP_INFO(node_->get_logger(), "fp_output.reconnect_delay : %f", reconnect_delay);

    std::string type_str;
    node_->get_parameter("fp_output.type", type_str);
    if (type_str == "tcp") {
        type = INPUT_TYPE::TCP;
    } else if (type_str == "serial") {
        type = INPUT_TYPE::SERIAL;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Input type has to be tcp or serial!");
        return false;
    }

    node_->get_parameter("fp_output.formats", formats);
    for (size_t i = 0; i < formats.size(); i++)
    	RCLCPP_INFO(node_->get_logger(), "fp_output.formats[%ld] : %s", i, formats[i].c_str());

    // Get parameters: port (required)
    node_->get_parameter("fp_output.port", port);
    RCLCPP_INFO(node_->get_logger(), "fp_output.port : %s", port.c_str());

    if (type == INPUT_TYPE::TCP) {
        node_->get_parameter("fp_output.ip", ip);
        RCLCPP_INFO(node_->get_logger(), "fp_output.ip : %s", ip.c_str());
    } else if (type == INPUT_TYPE::SERIAL) {
        node_->get_parameter("fp_output.baudrate", baudrate);
    RCLCPP_INFO(node_->get_logger(), "fp_output.baudrate : %d", baudrate);
    }

    return true;
}

CustomerInputParams::CustomerInputParams(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
    node_->declare_parameter("customer_input.speed_topic", "/fixposition/speed");
}

bool CustomerInputParams::LoadFromRos() {
    node_->get_parameter("customer_input.speed_topic", speed_topic);
    RCLCPP_INFO(node_->get_logger(), "customer_input.speed_topic : %s", speed_topic.c_str());
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
