/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Implementation of OdomConverterNode class
 */

/* LIBC/STL */

/* EXTERNAL */

/* PACKAGE */
#include "fixposition_odometry_converter_ros2/odom_converter_node.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

OdomConverterNode::OdomConverterNode(const rclcpp::NodeOptions& options) : Node("odom_converter", options) {
    // read parameters
    if (!params_.LoadFromRos(this->get_node_parameters_interface(), this->get_node_logging_interface())) {
        RCLCPP_ERROR(this->get_logger(), "Parameter Loading failed, shutting down...");
        rclcpp::shutdown();
    }

    // initialize the subscriber
    Subscribe();

    // initialize the publisher
    ws_pub_ = this->create_publisher<fixposition_driver_msgs::msg::Speed>(params_.fixposition_speed_topic, 10);
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

void OdomConverterNode::ConvertAndPublish(const std::vector<std::pair<bool, double>> speeds) {
    if (ws_pub_->get_subscription_count() > 0) {
        if (speeds.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "Speed vector has an invalid size");
            return;
        }
        fixposition_driver_msgs::msg::Speed msg;
        fixposition_driver_msgs::msg::WheelSensor sensor;
        sensor.location = "RC";
        sensor.vx_valid = speeds[0].first;
        sensor.vx = round(speeds[0].second * params_.multiplicative_factor);
        sensor.vy_valid = speeds[1].first;
        sensor.vy = round(speeds[1].second * params_.multiplicative_factor);
        sensor.vz_valid = speeds[2].first;
        sensor.vz = round(speeds[2].second * params_.multiplicative_factor);
        msg.sensors.push_back(sensor);
        ws_pub_->publish(msg);
    }
}

void OdomConverterNode::TwistWithCovCallback(const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg) {
    std::vector<std::pair<bool, double>> velocities;
    velocities.push_back({params_.use_x, msg->twist.linear.x});
    velocities.push_back({params_.use_y, msg->twist.linear.y});
    velocities.push_back({params_.use_z, msg->twist.linear.z});
    ConvertAndPublish(velocities);
}

void OdomConverterNode::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::vector<std::pair<bool, double>> velocities;
    velocities.push_back({params_.use_x, msg->twist.twist.linear.x});
    velocities.push_back({params_.use_y, msg->twist.twist.linear.y});
    velocities.push_back({params_.use_z, msg->twist.twist.linear.z});
    ConvertAndPublish(velocities);
}

void OdomConverterNode::TwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::vector<std::pair<bool, double>> velocities;
    velocities.push_back({params_.use_x, msg->linear.x});
    velocities.push_back({params_.use_y, msg->linear.y});
    velocities.push_back({params_.use_z, msg->linear.z});
    ConvertAndPublish(velocities);
}

/* ****************************************************************************************************************** */
}  // namespace fixposition

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fixposition::OdomConverterNode)
