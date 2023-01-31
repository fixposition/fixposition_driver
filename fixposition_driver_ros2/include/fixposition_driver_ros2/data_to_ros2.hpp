/**
 *  @file
 *  @brief Convert Data classes to ROS2 msgs
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_ROS2_DATA_TO_ROS2__
#define __FIXPOSITION_DRIVER_ROS2_DATA_TO_ROS2__

/* EXTERNAL */
#include <eigen3/Eigen/Core>

/* ROS */

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

/* FIXPOSITION */
#include <fixposition_driver_lib/converter/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

/* PACKAGE */
#include <fixposition_driver_ros2/msg/vrtk.hpp>

namespace fixposition {

/**
 * @brief Convert to ROS2 message time
 *
 * @param[in] input times::GpsTime
 * @return builtin_interfaces::msg::Time ros2 messge time
 */
inline builtin_interfaces::msg::Time GpsTimeToMsgTime(times::GpsTime input) {
    BOOST_POSIX::time_duration d = GpsTimeToPtime(input) - BOOST_POSIX::from_time_t(0);
    builtin_interfaces::msg::Time t;
    int64_t sec64 = d.total_seconds();
    if (sec64 < 0 || sec64 > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("time_duration is out of dual 32-bit range");
    t.sec = (uint32_t)sec64;
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
    t.nanosec = d.fractional_seconds();
#else
    t.nanosec = d.fractional_seconds() * 1000;
#endif
    return t;
}

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void ImuDataToMsg(const ImuData& data, sensor_msgs::msg::Imu& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void NavSatFixDataToMsg(const NavSatFixData& data, sensor_msgs::msg::NavSatFix& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void PoseWithCovDataToMsg(const PoseWithCovData& data, geometry_msgs::msg::PoseWithCovariance& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void TwistWithCovDataToMsg(const TwistWithCovData& data, geometry_msgs::msg::TwistWithCovariance& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdometryDataToMsg(const OdometryData& data, nav_msgs::msg::Odometry& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void VrtkDataToMsg(const VrtkData& data, fixposition_driver_ros2::msg::VRTK& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void TfDataToMsg(const TfData& data, geometry_msgs::msg::TransformStamped& msg);

}  // namespace fixposition

#endif
