// Wrapper to include ROS stuff withouth quoting the same warning suppressions all over
// --- ROS messages used in fusion_optim ---
#ifndef __FUSION_OPTIM_EXT_ROS2_MSGS_HPP__
#define __FUSION_OPTIM_EXT_ROS2_MSGS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#include <rcl/rcl.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <fixposition_driver_ros2/msg/speed.hpp>
#include <fixposition_driver_ros2/msg/vrtk.hpp>
#include <fixposition_driver_ros2/msg/nmea.hpp>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
// This header was deprecated as of ROS2 Humble, but is still required in order to support Foxy.
#include <tf2_eigen/tf2_eigen.h>
#endif

#pragma GCC diagnostic pop
#endif  // __FUSION_OPTIM_EXT_ROS2_MSGS_HPP__
