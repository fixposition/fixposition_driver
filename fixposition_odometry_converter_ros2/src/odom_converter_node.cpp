/**
 *  @file
 *  @brief Implementation of OdomConverterNode class
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
#include <fixposition_odometry_converter_ros2/odom_converter_node.hpp>

namespace fixposition {

OdomConverterNode::OdomConverterNode(const rclcpp::NodeOptions& options) : Node("odom_converter", options) {
    // read parameters
    if (!params_.LoadFromRos(this->get_node_parameters_interface(), this->get_node_logging_interface())) {
        RCLCPP_ERROR(this->get_logger(), "Parameter Loading failed, shutting down...");
        rclcpp::shutdown();
    }

    // initialize the subscriber
    Subscribe();

    // initialize the publisher
    ws_pub_ = this->create_publisher<fixposition_driver_ros2::msg::Speed>(params_.fixposition_speed_topic, 10);
}

void OdomConverterNode::Subscribe() {
    switch (params_.topic_type) {
        case VelTopicType::Twist:
            ws_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                params_.input_topic, 10, std::bind(&OdomConverterNode::TwistCallback, this, std::placeholders::_1));
            break;
        case VelTopicType::TwistWithCov:
            ws_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovariance>(
                params_.input_topic, 10,
                std::bind(&OdomConverterNode::TwistWithCovCallback, this, std::placeholders::_1));
            break;
        case VelTopicType::Odometry:
            ws_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                params_.input_topic, 10, std::bind(&OdomConverterNode::OdometryCallback, this, std::placeholders::_1));
            break;
    }
}

void OdomConverterNode::ConvertAndPublish(const double speed, const double angular, bool use_angular) {
    if (ws_pub_->get_subscription_count() > 0) {
        const int int_speed = round(speed * params_.multiplicative_factor);
        const int angular_speed = round(angular * params_.multiplicative_factor);
        fixposition_driver_ros2::msg::Speed msg;
        msg.speeds.push_back(int_speed);
        if (params_.use_angular) {
            msg.speeds.push_back(angular_speed);
        }
        ws_pub_->publish(msg);
    }
}

void OdomConverterNode::TwistWithCovCallback(const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg) {
    ConvertAndPublish(msg->twist.linear.x, msg->twist.angular.z, params_.use_angular);
}

void OdomConverterNode::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ConvertAndPublish(msg->twist.twist.linear.x, msg->twist.twist.angular.z, params_.use_angular);
}

void OdomConverterNode::TwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    ConvertAndPublish(msg->linear.x, msg->angular.z, params_.use_angular);
}

}  // namespace fixposition

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fixposition::OdomConverterNode)