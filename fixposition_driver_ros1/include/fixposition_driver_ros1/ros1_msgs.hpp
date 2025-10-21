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
#include <fixposition_driver_msgs/FusionEpoch.h>
#include <fixposition_driver_msgs/NmeaEpoch.h>
#include <fixposition_driver_msgs/ParserMsg.h>
#include <fixposition_driver_msgs/Speed.h>
// -Extras
#include <fixposition_driver_msgs/CovWarn.h>
// - FP-A
#include <fixposition_driver_msgs/FpaConsts.h>
#include <fixposition_driver_msgs/FpaEoe.h>
#include <fixposition_driver_msgs/FpaGnssant.h>
#include <fixposition_driver_msgs/FpaGnsscorr.h>
#include <fixposition_driver_msgs/FpaImu.h>
#include <fixposition_driver_msgs/FpaImubias.h>
#include <fixposition_driver_msgs/FpaLlh.h>
#include <fixposition_driver_msgs/FpaOdomenu.h>
#include <fixposition_driver_msgs/FpaOdometry.h>
#include <fixposition_driver_msgs/FpaOdomsh.h>
#include <fixposition_driver_msgs/FpaOdomstatus.h>
#include <fixposition_driver_msgs/FpaText.h>
#include <fixposition_driver_msgs/FpaTp.h>
// - NMEA
#include <fixposition_driver_msgs/NmeaGga.h>
#include <fixposition_driver_msgs/NmeaGll.h>
#include <fixposition_driver_msgs/NmeaGsa.h>
#include <fixposition_driver_msgs/NmeaGst.h>
#include <fixposition_driver_msgs/NmeaGsv.h>
#include <fixposition_driver_msgs/NmeaHdt.h>
#include <fixposition_driver_msgs/NmeaRmc.h>
#include <fixposition_driver_msgs/NmeaVtg.h>
#include <fixposition_driver_msgs/NmeaZda.h>
// - NOV-B
#include <fixposition_driver_msgs/NovbInspvax.h>
#include <fixposition_driver_msgs/NovbHeading2.h>

#pragma GCC diagnostic pop
#endif  // __FIXPOSITION_DRIVER_ROS1_ROS_MSGS_HPP__
