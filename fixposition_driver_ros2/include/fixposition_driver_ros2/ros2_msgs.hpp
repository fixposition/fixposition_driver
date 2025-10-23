// Wrapper to suppress warnings from ROS headers
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
#include <fixposition_driver_msgs/msg/fusion_epoch.hpp>
#include <fixposition_driver_msgs/msg/nmea_epoch.hpp>
#include <fixposition_driver_msgs/msg/parser_msg.hpp>
#include <fixposition_driver_msgs/msg/speed.hpp>
// - Extras
#include <fixposition_driver_msgs/msg/cov_warn.hpp>
// - FP-A
#include <fixposition_driver_msgs/msg/fpa_consts.hpp>
#include <fixposition_driver_msgs/msg/fpa_eoe.hpp>
#include <fixposition_driver_msgs/msg/fpa_gnssant.hpp>
#include <fixposition_driver_msgs/msg/fpa_gnsscorr.hpp>
#include <fixposition_driver_msgs/msg/fpa_imu.hpp>
#include <fixposition_driver_msgs/msg/fpa_imubias.hpp>
#include <fixposition_driver_msgs/msg/fpa_llh.hpp>
#include <fixposition_driver_msgs/msg/fpa_odomenu.hpp>
#include <fixposition_driver_msgs/msg/fpa_odometry.hpp>
#include <fixposition_driver_msgs/msg/fpa_odomsh.hpp>
#include <fixposition_driver_msgs/msg/fpa_odomstatus.hpp>
#include <fixposition_driver_msgs/msg/fpa_text.hpp>
#include <fixposition_driver_msgs/msg/fpa_tp.hpp>
// - NMEA
#include <fixposition_driver_msgs/msg/nmea_gga.hpp>
#include <fixposition_driver_msgs/msg/nmea_gll.hpp>
#include <fixposition_driver_msgs/msg/nmea_gsa.hpp>
#include <fixposition_driver_msgs/msg/nmea_gst.hpp>
#include <fixposition_driver_msgs/msg/nmea_gsv.hpp>
#include <fixposition_driver_msgs/msg/nmea_hdt.hpp>
#include <fixposition_driver_msgs/msg/nmea_rmc.hpp>
#include <fixposition_driver_msgs/msg/nmea_vtg.hpp>
#include <fixposition_driver_msgs/msg/nmea_zda.hpp>
// - NOV-B
#include <fixposition_driver_msgs/msg/novb_heading2.hpp>
#include <fixposition_driver_msgs/msg/novb_inspvax.hpp>

// Shortcut
namespace fpmsgs = fixposition_driver_msgs::msg;

#pragma GCC diagnostic pop
#endif  // __FIXPOSITION_DRIVER_ROS2_ROS2_MSGS_HPP__
