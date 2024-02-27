/**
 *  @file
 *  @brief Implementation of OdomConverter class
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
#include <fixposition_odometry_converter/odom_converter.hpp>

namespace fixposition {
OdomConverter::OdomConverter(ros::NodeHandle* nh) : nh_(*nh) {
    // read parameters
    if (!params_.LoadFromRos("/fixposition_odometry_converter")) {
        ROS_ERROR_STREAM("Parameter Loading failed, shutting down...");
        ros::shutdown();
    }

    // initialize the subscriber
    Subscribe();

    // initialize the publisher
    ws_pub_ = nh_.advertise<fixposition_driver_ros1::Speed>(params_.fixposition_speed_topic, 1);
}

void OdomConverter::Subscribe() {
    switch (params_.topic_type) {
        case VelTopicType::Twist:
            ws_sub_ = nh_.subscribe<geometry_msgs::Twist>(params_.input_topic, 1, &OdomConverter::TwistCallback, this);
            break;
        case VelTopicType::TwistWithCov:
            ws_sub_ = nh_.subscribe<geometry_msgs::TwistWithCovariance>(params_.input_topic, 1,
                                                                        &OdomConverter::TwistWithCovCallback, this);
            break;
        case VelTopicType::Odometry:
            ws_sub_ = nh_.subscribe<nav_msgs::Odometry>(params_.input_topic, 1, &OdomConverter::OdometryCallback, this);
            break;
    }
}

void OdomConverter::ConvertAndPublish(const std::vector<double> speeds) {
    if (ws_pub_.getNumSubscribers() > 0) {
        fixposition_driver_ros1::Speed msg;
        fixposition_driver_ros1::WheelSensor sensor;
        sensor.location = "RC";
        for (const auto speed : speeds) {
            const int int_speed = round(speed * params_.multiplicative_factor);
            sensor.speeds.push_back(int_speed);
        }
        msg.sensors.push_back(sensor);
        ws_pub_.publish(msg);
    }
}

void OdomConverter::TwistWithCovCallback(const geometry_msgs::TwistWithCovarianceConstPtr& msg) {
    std::vector<double> velocities;
    if (params_.use_dimensions >= 1) {
        velocities.push_back(msg->twist.linear.x);
    }
    if (params_.use_dimensions >= 2) {
        velocities.push_back(msg->twist.linear.y);
    }
    if (params_.use_dimensions >= 3) {
        velocities.push_back(msg->twist.linear.z);
    }
    ConvertAndPublish(velocities);
}

void OdomConverter::OdometryCallback(const nav_msgs::OdometryConstPtr& msg) {
    std::vector<double> velocities;
    if (params_.use_dimensions >= 1) {
        velocities.push_back(msg->twist.twist.linear.x);
    }
    if (params_.use_dimensions >= 2) {
        velocities.push_back(msg->twist.twist.linear.y);
    }
    if (params_.use_dimensions >= 3) {
        velocities.push_back(msg->twist.twist.linear.z);
    }
    ConvertAndPublish(velocities);
}

void OdomConverter::TwistCallback(const geometry_msgs::TwistConstPtr& msg) {
    std::vector<double> velocities;
    if (params_.use_dimensions >= 1) {
        velocities.push_back(msg->linear.x);
    }
    if (params_.use_dimensions >= 2) {
        velocities.push_back(msg->linear.y);
    }
    if (params_.use_dimensions >= 3) {
        velocities.push_back(msg->linear.z);
    }
    ConvertAndPublish(velocities);
}

}  // namespace fixposition
