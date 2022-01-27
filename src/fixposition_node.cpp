/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /  
 *   /  /\  \  
 *  /__/  \__\  Fixposition AG
 * 
 * @file fixposition_node.cpp
 * @author Kailin Huang (kailin.huang@fixposition.com)
 * @brief 
 * @date 2022-01-26
 * 
 */

#include <ros/console.h>
#include <ros/ros.h>

#include <fixposition_driver/fixposition_driver.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fixposition_converter");
    ros::NodeHandle node_handle;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    fixposition::FixpositionDriver fixposition_out(&node_handle);
    ROS_DEBUG("Starting node...");
    fixposition_out.Run();
    ros::waitForShutdown();
    ROS_DEBUG("Exiting.");
    return 0;
}
