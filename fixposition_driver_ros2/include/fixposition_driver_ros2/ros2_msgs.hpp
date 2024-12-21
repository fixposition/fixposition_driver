// Wrapper to include ROS2 stuff without quoting the same warning suppressions all over
#ifndef __FIXPOSITION_DRIVER_ROS2_ROS2_MSGS_HPP__
#define __FIXPOSITION_DRIVER_ROS2_ROS2_MSGS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

// Standard ROS messages
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// RTCM
#include <rtcm_msgs/msg/message.hpp>

// Fixposition ROS messages
// - Generic
#include <fixposition_driver_ros2/msg/gnsssats.hpp>
#include <fixposition_driver_ros2/msg/nmea.hpp>
#include <fixposition_driver_ros2/msg/speed.hpp>
// - Extras
#include <fixposition_driver_ros2/msg/covwarn.hpp>
// - FP-A
#include <fixposition_driver_ros2/msg/eoe.hpp>
#include <fixposition_driver_ros2/msg/gnssant.hpp>
#include <fixposition_driver_ros2/msg/gnsscorr.hpp>
#include <fixposition_driver_ros2/msg/imubias.hpp>
#include <fixposition_driver_ros2/msg/llh.hpp>
#include <fixposition_driver_ros2/msg/odomenu.hpp>
#include <fixposition_driver_ros2/msg/odometry.hpp>
#include <fixposition_driver_ros2/msg/odomsh.hpp>
#include <fixposition_driver_ros2/msg/odomstatus.hpp>
#include <fixposition_driver_ros2/msg/text.hpp>
#include <fixposition_driver_ros2/msg/tp.hpp>
// - NMEA
#include <fixposition_driver_ros2/msg/gngsa.hpp>
#include <fixposition_driver_ros2/msg/gpgga.hpp>
#include <fixposition_driver_ros2/msg/gpgll.hpp>
#include <fixposition_driver_ros2/msg/gpgst.hpp>
#include <fixposition_driver_ros2/msg/gphdt.hpp>
#include <fixposition_driver_ros2/msg/gprmc.hpp>
#include <fixposition_driver_ros2/msg/gpvtg.hpp>
#include <fixposition_driver_ros2/msg/gpzda.hpp>
#include <fixposition_driver_ros2/msg/gxgsv.hpp>

#pragma GCC diagnostic pop
#endif  // __FIXPOSITION_DRIVER_ROS2_ROS2_MSGS_HPP__
