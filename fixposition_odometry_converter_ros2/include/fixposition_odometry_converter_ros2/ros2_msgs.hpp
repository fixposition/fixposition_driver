// Wrapper to include ROS2 stuff without quoting the same warning suppressions all over
#ifndef __FIXPOSITION_ODOMETRY_CONVERTER_ROS2_ROS2_MSGS_HPP__
#define __FIXPOSITION_ODOMETRY_CONVERTER_ROS2_ROS2_MSGS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#include <fixposition_driver_msgs/msg/speed.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#pragma GCC diagnostic pop
#endif  // __FIXPOSITION_ODOMETRY_CONVERTER_ROS2_ROS2_MSGS_HPP__
