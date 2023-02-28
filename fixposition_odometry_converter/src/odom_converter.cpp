/**
 *  @file
 *  @brief Implementation of OdomConverter class
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
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

void OdomConverter::ConvertAndPublish(const double speed, const double angular, bool use_angular) {
    if (ws_pub_.getNumSubscribers() > 0) {
        const int int_speed = round(speed * params_.multiplicative_factor);
        const int angular_speed = round(angular * params_.multiplicative_factor);
        fixposition_driver_ros1::Speed msg;
        msg.speeds.push_back(int_speed);
        if (params_.use_angular) {
            msg.speeds.push_back(angular_speed);
        }
        ws_pub_.publish(msg);
    }
}

void OdomConverter::TwistWithCovCallback(const geometry_msgs::TwistWithCovarianceConstPtr& msg) {
    ConvertAndPublish(msg->twist.linear.x, msg->twist.angular.z, params_.use_angular);
}

void OdomConverter::OdometryCallback(const nav_msgs::OdometryConstPtr& msg) {
    ConvertAndPublish(msg->twist.twist.linear.x, msg->twist.twist.angular.z, params_.use_angular);
}

void OdomConverter::TwistCallback(const geometry_msgs::TwistConstPtr& msg) {
    ConvertAndPublish(msg->linear.x, msg->angular.z, params_.use_angular);
}

}  // namespace fixposition
