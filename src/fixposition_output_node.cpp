/**
 * @file fixposition_output_node.cpp
 * @author Andreea Lutac (andreea.lutac@fixposition.ch)
 * @brief
 * version 0.1
 * @date 2020-11-09
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <ros/console.h>
#include <ros/ros.h>

#include "fixposition_output.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "fixposition_converter");
    ros::NodeHandle node_handle;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    FixpositionOutput fixposition_out(&node_handle);
    ROS_DEBUG("Starting node...");
    ros_io.Run();
    ros::waitForShutdown();
    ROS_DEBUG("Exiting.");
    return 0;
}
