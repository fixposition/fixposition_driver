// Wrapper to include ROS2 stuff without quoting the same warning suppressions all over
#ifndef __ROS2_DRIVER_CONVERTER_MSGS_HPP__
#define __ROS2_DRIVER_CONVERTER_MSGS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include <fixposition_driver_ros2/msg/speed.hpp>

#pragma GCC diagnostic pop
#endif  // __ROS2_DRIVER_CONVERTER_MSGS_HPP__
