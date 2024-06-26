// Wrapper to include ROS stuff without quoting the same warning suppressions all over
#ifndef __ROS1_DRIVER_CONVERTER_MSGS_HPP__
#define __ROS1_DRIVER_CONVERTER_MSGS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <fixposition_driver_ros1/Speed.h>

#pragma GCC diagnostic pop
#endif  // __ROS1_DRIVER_CONVERTER_MSGS_HPP__
