/**
 *  @file
 *  @brief Convert Data classes to ROS1 msgs
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

#ifndef __FIXPOSITION_DRIVER_ROS1_DATA_TO_ROS1__
#define __FIXPOSITION_DRIVER_ROS1_DATA_TO_ROS1__

/* FIXPOSITION DRIVER LIB */
#include <fixposition_driver_lib/messages/msg_data.hpp>
#include <fixposition_driver_lib/fixposition_driver.hpp>

/* PACKAGE */
#include <fixposition_driver_ros1/fixposition_driver_node.hpp>

namespace fixposition {
/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void ImuDataToMsg(const ImuData& data, sensor_msgs::Imu& msg);

/**
 * @brief 
 * 
 * @param[in] data
 * @param[in] msg
 */
void NavSatStatusDataToMsg(const NavSatStatusData& data, sensor_msgs::NavSatStatus& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void NavSatFixDataToMsg(const NavSatFixData& data, sensor_msgs::NavSatFix& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void PoseWithCovDataToMsg(const PoseWithCovData& data, geometry_msgs::PoseWithCovariance& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void TwistWithCovDataToMsg(const TwistWithCovData& data, geometry_msgs::TwistWithCovariance& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdometryDataToMsg(const OdometryData& data, nav_msgs::Odometry& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdomDataToMsg(const fixposition::FP_ODOMETRY& data, nav_msgs::Odometry& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void VrtkDataToMsg(const VrtkData& data, fixposition_driver_ros1::VRTK& msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void TfDataToMsg(const TfData& data, geometry_msgs::TransformStamped& msg);

}  // namespace fixposition

#endif
