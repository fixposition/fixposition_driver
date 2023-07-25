/**
 *  @file
 *  @brief Implementation of Parameter Loading for the odometry parameters
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * 
 * Port to ROS 2 by Husarion
 * \endverbatim
 *
 */

/* PACKAGE */
#include <fixposition_odometry_converter_ros2/params.hpp>

namespace fixposition {

bool OdomInputParams::LoadFromRos(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& param_itf,
                                  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_itf) {
    param_itf->declare_parameter("fixposition_speed_topic", rclcpp::ParameterValue("/fixposition/speed"));
    fixposition_speed_topic = param_itf->get_parameter("fixposition_speed_topic").as_string();

    param_itf->declare_parameter("multiplicative_factor", rclcpp::ParameterValue(1000));
    multiplicative_factor = param_itf->get_parameter("multiplicative_factor").as_int();

    param_itf->declare_parameter("use_angular", rclcpp::ParameterValue(false));
    use_angular = param_itf->get_parameter("use_angular").as_bool();

    param_itf->declare_parameter("input_topic", rclcpp::PARAMETER_STRING);
    try {
        input_topic = param_itf->get_parameter("input_topic").as_string();
    } catch (const rclcpp::exceptions::ParameterUninitializedException& e) {
        RCLCPP_ERROR(logging_itf->get_logger(), "Couldn't read the input topic name.");
        return false;
    }

    param_itf->declare_parameter("topic_type", rclcpp::PARAMETER_STRING);
    std::string topic_type_string;
    try {
        topic_type_string = param_itf->get_parameter("topic_type").as_string();
    } catch (const rclcpp::exceptions::ParameterUninitializedException& e) {
        RCLCPP_ERROR(logging_itf->get_logger(), "Couldn't read the input topic type.");
        return false;
    }

    if (topic_type_string == "Twist") {
        topic_type = VelTopicType::Twist;
    } else if (topic_type_string == "TwistWithCov") {
        topic_type = VelTopicType::TwistWithCov;
    } else if (topic_type_string == "Odometry") {
        topic_type = VelTopicType::Odometry;
    } else {
        RCLCPP_ERROR(logging_itf->get_logger(), "Topic type is not supported.");
        return false;
    }

    return true;
}

}  // namespace fixposition
