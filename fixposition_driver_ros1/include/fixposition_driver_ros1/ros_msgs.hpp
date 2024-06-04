// Wrapper to include ROS stuff withouth quoting the same warning suppressions all over
// --- ROS messages used in fusion_optim ---
#ifndef __FUSION_OPTIM_EXT_ROS_MSGS_HPP__
#define __FUSION_OPTIM_EXT_ROS_MSGS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <fixposition_driver_ros1/Speed.h>
#include <fixposition_driver_ros1/VRTK.h>
#include <fixposition_driver_ros1/NMEA.h>

#include <eigen_conversions/eigen_msg.h>

#pragma GCC diagnostic pop
#endif  // __FUSION_OPTIM_EXT_ROS_MSGS_HPP__
