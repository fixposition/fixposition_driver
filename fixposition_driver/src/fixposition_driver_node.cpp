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
#include "rclcpp/rclcpp.hpp"

/* PACKAGE */
#include "fixposition_driver/fixposition_driver.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fixposition_driver");
    fixposition::FixpositionDriver fixposition_out(node);
    RCLCPP_DEBUG(node->get_logger(), "Starting node...");
    fixposition_out.Run();
    rclcpp::spin(node);
    RCLCPP_DEBUG(node->get_logger(), "Exiting.");
    return 0;
}
