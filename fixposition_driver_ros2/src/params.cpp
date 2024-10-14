/**
 *  @file
 *  @brief Implementation of Parameter Loading
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

/* PACKAGE */
#include <fixposition_driver_ros2/params.hpp>

namespace fixposition {

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node> node, const std::string& ns, FpOutputParams& params) {
    const std::string RATE = ns + ".rate";
    const std::string RECONNECT_DELAY = ns + ".reconnect_delay";
    const std::string TYPE = ns + ".type";
    const std::string FORMATS = ns + ".formats";
    const std::string QOS_TYPE = ns + ".qos_type";
    const std::string IP = ns + ".ip";
    const std::string PORT = ns + ".port";
    const std::string BAUDRATE = ns + ".baudrate";
    const std::string COV_WARNING = ns + ".cov_warning";
    const std::string NAV2_MODE = ns + ".nav2_mode";

    node->declare_parameter(RATE, 100);
    node->declare_parameter(RECONNECT_DELAY, 5.0);
    node->declare_parameter(TYPE, "tcp");
    node->declare_parameter(FORMATS, std::vector<std::string>());
    node->declare_parameter(QOS_TYPE, "sensor_short");
    node->declare_parameter(PORT, "21000");
    node->declare_parameter(IP, "127.0.0.1");
    node->declare_parameter(BAUDRATE, 115200);
    node->declare_parameter(COV_WARNING, false);
    node->declare_parameter(NAV2_MODE, false);
    // read parameters
    if (node->get_parameter(RATE, params.rate)) {
        RCLCPP_INFO(node->get_logger(), "%s : %d", RATE.c_str(), params.rate);
    } else {
        RCLCPP_WARN(node->get_logger(), "Using Default %s : %d", RATE.c_str(), params.rate);
    }
    if (node->get_parameter(RECONNECT_DELAY, params.reconnect_delay)) {
        RCLCPP_INFO(node->get_logger(), "%s : %f", RECONNECT_DELAY.c_str(), params.reconnect_delay);
    } else {
        RCLCPP_WARN(node->get_logger(), "%s : %f", RECONNECT_DELAY.c_str(), params.reconnect_delay);
    }
    if (node->get_parameter(QOS_TYPE, params.qos_type)) {
        RCLCPP_INFO(node->get_logger(), "%s : %s", QOS_TYPE.c_str(), params.qos_type.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "%s : %s", QOS_TYPE.c_str(), params.qos_type.c_str());
    }
    if (node->get_parameter(COV_WARNING, params.cov_warning)) {
        RCLCPP_INFO(node->get_logger(), "%s : %d", COV_WARNING.c_str(), params.cov_warning);
    } else {
        RCLCPP_WARN(node->get_logger(), "%s : %d", COV_WARNING.c_str(), params.cov_warning);
    }
    if (node->get_parameter(NAV2_MODE, params.nav2_mode)) {
        RCLCPP_INFO(node->get_logger(), "%s : %d", NAV2_MODE.c_str(), params.nav2_mode);
    } else {
        RCLCPP_WARN(node->get_logger(), "%s : %d", NAV2_MODE.c_str(), params.nav2_mode);
    }

    std::string type_str;
    node->get_parameter(TYPE, type_str);
    if (type_str == "tcp") {
        params.type = INPUT_TYPE::TCP;
    } else if (type_str == "serial") {
        params.type = INPUT_TYPE::SERIAL;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Input type has to be tcp or serial!");
        return false;
    }

    node->get_parameter(FORMATS, params.formats);
    for (size_t i = 0; i < params.formats.size(); i++) {
        RCLCPP_INFO(node->get_logger(), "%s[%ld] : %s", FORMATS.c_str(), i, params.formats.at(i).c_str());
    }

    // Get parameters: port (required)
    if (node->get_parameter(PORT, params.port)) {
        RCLCPP_INFO(node->get_logger(), "%s : %s", PORT.c_str(), params.port.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "Using Default %s : %s", PORT.c_str(), params.port.c_str());
    }

    if (params.type == INPUT_TYPE::TCP) {
        if (node->get_parameter(IP, params.ip)) {
            RCLCPP_INFO(node->get_logger(), "%s : %s", IP.c_str(), params.ip.c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "Using Default %s : %s", IP.c_str(), params.ip.c_str());
        }
    } else if (params.type == INPUT_TYPE::SERIAL) {
        if (node->get_parameter(BAUDRATE, params.baudrate)) {
            RCLCPP_INFO(node->get_logger(), "%s : %d", BAUDRATE.c_str(), params.baudrate);
        } else {
            RCLCPP_WARN(node->get_logger(), "Using Default %s : %d", BAUDRATE.c_str(), params.baudrate);
        }
    }

    return true;
}

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node> node, const std::string& ns, CustomerInputParams& params) {
    const std::string SPEED_TOPIC = ns + ".speed_topic";
    node->declare_parameter(SPEED_TOPIC, "/fixposition/speed");
    node->get_parameter(SPEED_TOPIC, params.speed_topic);
    RCLCPP_INFO(node->get_logger(), "%s : %s", SPEED_TOPIC.c_str(), params.speed_topic.c_str());

    const std::string RTCM_TOPIC = ns + ".rtcm_topic";
    node->declare_parameter(RTCM_TOPIC, "/fixposition/rtcm");
    node->get_parameter(RTCM_TOPIC, params.rtcm_topic);
    RCLCPP_INFO(node->get_logger(), "%s : %s", RTCM_TOPIC.c_str(), params.rtcm_topic.c_str());

    return true;
}

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node> node, FixpositionDriverParams& params) {
    bool ok = true;

    ok &= LoadParamsFromRos2(node, "fp_output", params.fp_output);
    ok &= LoadParamsFromRos2(node, "customer_input", params.customer_input);

    return ok;
}

}  // namespace fixposition
