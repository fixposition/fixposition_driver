/**
 *  @file
 *  @brief Implementation of Parameter Loading for the odometry parameters
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* ROS */
#include <ros/console.h>
#include <ros/ros.h>

/* PACKAGE */
#include <fixposition_odometry_converter/params.hpp>

namespace fixposition {

bool OdomInputParams::LoadFromRos(const std::string &ns) {

    if (!ros::param::get(ns + "/fixposition_speed_topic", fixposition_speed_topic)) fixposition_speed_topic = "/fixposition/speed";
    if (!ros::param::get(ns + "/multiplicative_factor", multiplicative_factor)) multiplicative_factor = 1000;
    if (!ros::param::get(ns + "/use_angular", use_angular)) use_angular = false;
    if (!ros::param::get(ns + "/input_topic", input_topic)) {
        ROS_ERROR("Couldn't read the input topic name.");
        return false;
    }
    std::string topic_type_string;
    if (!ros::param::get(ns + "/topic_type", topic_type_string)) {
        ROS_ERROR("Couldn't read the input topic type.");
        return false;
    }
    if (topic_type_string == "Twist") {
        topic_type = VelTopicType::Twist;
    } else if (topic_type_string == "TwistWithCov") {
        topic_type = VelTopicType::TwistWithCov;
    } else if (topic_type_string == "Odometry") {
        topic_type = VelTopicType::Odometry;
    } else {
        ROS_ERROR("Topic type is not supported.");
        return false;
    }
    
    return true;
}

}  // namespace fixposition
