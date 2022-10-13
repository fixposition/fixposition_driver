/**
 *  @file
 *  @brief Main function for the odometry converter ros node
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
#include <fixposition_odometry_converter/odom_converter.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fixposition_odometry_converter");
    ros::NodeHandle node_handle;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    fixposition::OdomConverter odom_converter(&node_handle);
    ROS_DEBUG("Starting fixposition_odometry_converter node...");
    ros::spin();
    ROS_DEBUG("Exiting.");
    return 0;
}
