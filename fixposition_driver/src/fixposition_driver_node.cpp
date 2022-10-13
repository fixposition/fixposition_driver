/**
 *  @file
 *  @brief Main function for the fixposition driver ros node
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
#include <fixposition_driver/fixposition_driver.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fixposition_driver");
    ros::NodeHandle node_handle;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    fixposition::FixpositionDriver fixposition_out(&node_handle);
    ROS_DEBUG("Starting node...");
    fixposition_out.Run();
    ros::waitForShutdown();
    ROS_DEBUG("Exiting.");
    return 0;
}
