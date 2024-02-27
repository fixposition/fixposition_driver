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

void OdomConverter::ConvertAndPublish(const std::vector<std::pair<bool, double>> speeds) {
    if (ws_pub_.getNumSubscribers() > 0) {
        if (speeds.size() != 3) {
            ROS_ERROR("Speed vector has an invalid size!");
            return;
        }
        fixposition_driver_ros1::Speed msg;
        fixposition_driver_ros1::WheelSensor sensor;
        sensor.location = "RC";
        sensor.vx_valid = speeds[0].first;
        sensor.vx = round(speeds[0].second * params_.multiplicative_factor);
        sensor.vy_valid = speeds[1].first;
        sensor.vy = round(speeds[1].second * params_.multiplicative_factor);
        sensor.vz_valid = speeds[2].first;
        sensor.vz = round(speeds[2].second * params_.multiplicative_factor);
        msg.sensors.push_back(sensor);
        ws_pub_.publish(msg);
    }
}

void OdomConverter::TwistWithCovCallback(const geometry_msgs::TwistWithCovarianceConstPtr& msg) {
    std::vector<std::pair<bool, double>> velocities;
    velocities.push_back({params_.use_x, msg->twist.linear.x});
    velocities.push_back({params_.use_y, msg->twist.linear.y});
    velocities.push_back({params_.use_z, msg->twist.linear.z});
    ConvertAndPublish(velocities);
}

void OdomConverter::OdometryCallback(const nav_msgs::OdometryConstPtr& msg) {
    std::vector<std::pair<bool, double>> velocities;
    velocities.push_back({params_.use_x, msg->twist.twist.linear.x});
    velocities.push_back({params_.use_y, msg->twist.twist.linear.y});
    velocities.push_back({params_.use_z, msg->twist.twist.linear.z});
    ConvertAndPublish(velocities);
}

void OdomConverter::TwistCallback(const geometry_msgs::TwistConstPtr& msg) {
    std::vector<std::pair<bool, double>> velocities;
    velocities.push_back({params_.use_x, msg->linear.x});
    velocities.push_back({params_.use_y, msg->linear.y});
    velocities.push_back({params_.use_z, msg->linear.z});
    ConvertAndPublish(velocities);
}

}  // namespace fixposition
