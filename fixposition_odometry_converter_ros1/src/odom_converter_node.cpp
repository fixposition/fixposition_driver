/**
 *  @file
 *  @brief Main function for the odometry converter ros node
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

/* ROS */
#include <ros/console.h>
#include <ros/ros.h>

/* PACKAGE */
#include <fixposition_odometry_converter_ros1/odom_converter.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "fixposition_odometry_converter_ros1");
    ros::NodeHandle node_handle;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    fixposition::OdomConverter odom_converter(&node_handle);
    ROS_DEBUG("Starting fixposition_odometry_converter_ros1 node...");
    ros::spin();
    ROS_DEBUG("Exiting.");
    return 0;
}
