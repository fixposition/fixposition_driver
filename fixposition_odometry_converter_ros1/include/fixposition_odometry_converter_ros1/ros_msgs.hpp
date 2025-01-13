// Wrapper to include ROS stuff without quoting the same warning suppressions all over
#ifndef __FIXPOSITION_ODOMETRY_CONVERTER_ROS1_ROS_MSGS_HPP__
#define __FIXPOSITION_ODOMETRY_CONVERTER_ROS1_ROS_MSGS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#include <fixposition_driver_msgs/Speed.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>

#pragma GCC diagnostic pop
#endif  // __FIXPOSITION_ODOMETRY_CONVERTER_ROS1_ROS_MSGS_HPP__
