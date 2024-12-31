// Wrapper to suppress warnings from ROS headers
#ifndef __FIXPOSITION_DRIVER_ROS1_ROS_MSGS_HPP__
#define __FIXPOSITION_DRIVER_ROS1_ROS_MSGS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

// Standard ROS messages
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8MultiArray.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// RTCM
#include <rtcm_msgs/Message.h>

// Fixposition ROS messages
// - Generic
#include <fixposition_driver_ros1/GnssSats.h>
#include <fixposition_driver_ros1/NMEA.h>
#include <fixposition_driver_ros1/Speed.h>
// -Extras
#include <fixposition_driver_ros1/CovWarn.h>
// - FP-A
#include <fixposition_driver_ros1/eoe.h>
#include <fixposition_driver_ros1/gnssant.h>
#include <fixposition_driver_ros1/gnsscorr.h>
#include <fixposition_driver_ros1/imubias.h>
#include <fixposition_driver_ros1/llh.h>
#include <fixposition_driver_ros1/odomenu.h>
#include <fixposition_driver_ros1/odometry.h>
#include <fixposition_driver_ros1/odomsh.h>
#include <fixposition_driver_ros1/odomstatus.h>
#include <fixposition_driver_ros1/text.h>
#include <fixposition_driver_ros1/tp.h>
// - NMEA
#include <eigen_conversions/eigen_msg.h>
#include <fixposition_driver_ros1/gngsa.h>
#include <fixposition_driver_ros1/gpgga.h>
#include <fixposition_driver_ros1/gpgll.h>
#include <fixposition_driver_ros1/gpgst.h>
#include <fixposition_driver_ros1/gphdt.h>
#include <fixposition_driver_ros1/gprmc.h>
#include <fixposition_driver_ros1/gpvtg.h>
#include <fixposition_driver_ros1/gpzda.h>
#include <fixposition_driver_ros1/gxgsv.h>

#pragma GCC diagnostic pop
#endif  // __FIXPOSITION_DRIVER_ROS1_ROS_MSGS_HPP__
