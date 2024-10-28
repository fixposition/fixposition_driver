// Wrapper to include ROS stuff without quoting the same warning suppressions all over
#ifndef __ROS1_DRIVER_ROS_MSGS_HPP__
#define __ROS1_DRIVER_ROS_MSGS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/UInt8MultiArray.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Include generic
#include <fixposition_driver_ros1/Speed.h>
#include <fixposition_driver_ros1/GnssSats.h>
#include <fixposition_driver_ros1/NMEA.h>

// Include RTCM
#include <rtcm_msgs/Message.h>

// Include extras
#include <fixposition_driver_ros1/CovWarn.h>

// Include FP-A
#include <fixposition_driver_ros1/gnssant.h>
#include <fixposition_driver_ros1/gnsscorr.h>
#include <fixposition_driver_ros1/imubias.h>
#include <fixposition_driver_ros1/eoe.h>
#include <fixposition_driver_ros1/llh.h>
#include <fixposition_driver_ros1/odomenu.h>
#include <fixposition_driver_ros1/odometry.h>
#include <fixposition_driver_ros1/odomsh.h>
#include <fixposition_driver_ros1/odomstatus.h>
#include <fixposition_driver_ros1/text.h>
#include <fixposition_driver_ros1/tp.h>

// Include NMEA
#include <fixposition_driver_ros1/gpgga.h>
#include <fixposition_driver_ros1/gpgll.h>
#include <fixposition_driver_ros1/gngsa.h>
#include <fixposition_driver_ros1/gpgst.h>
#include <fixposition_driver_ros1/gxgsv.h>
#include <fixposition_driver_ros1/gphdt.h>
#include <fixposition_driver_ros1/gprmc.h>
#include <fixposition_driver_ros1/gpvtg.h>
#include <fixposition_driver_ros1/gpzda.h>

#include <eigen_conversions/eigen_msg.h>

#pragma GCC diagnostic pop
#endif  // __ROS1_DRIVER_ROS_MSGS_HPP__
