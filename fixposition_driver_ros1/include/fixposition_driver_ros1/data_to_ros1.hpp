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
void FpToRosMsg( const OdometryData& data, ros::Publisher& pub);
void FpToRosMsg(      const ImuData& data, ros::Publisher& pub);
void FpToRosMsg(       const FP_EOE& data, ros::Publisher& pub);
void FpToRosMsg(   const FP_GNSSANT& data, ros::Publisher& pub);
void FpToRosMsg(  const FP_GNSSCORR& data, ros::Publisher& pub);
void FpToRosMsg(   const FP_IMUBIAS& data, ros::Publisher& pub);
void FpToRosMsg(       const FP_LLH& data, ros::Publisher& pub);
void FpToRosMsg(   const FP_ODOMENU& data, ros::Publisher& pub);
void FpToRosMsg(  const FP_ODOMETRY& data, ros::Publisher& pub);
void FpToRosMsg(    const FP_ODOMSH& data, ros::Publisher& pub);
void FpToRosMsg(const FP_ODOMSTATUS& data, ros::Publisher& pub);
void FpToRosMsg(        const FP_TP& data, ros::Publisher& pub);
void FpToRosMsg(      const FP_TEXT& data, ros::Publisher& pub);

void FpToRosMsg(const GP_GGA& data, ros::Publisher& pub);
void FpToRosMsg(const GP_GLL& data, ros::Publisher& pub);
void FpToRosMsg(const GN_GSA& data, ros::Publisher& pub);
void FpToRosMsg(const GP_GST& data, ros::Publisher& pub);
void FpToRosMsg(const GX_GSV& data, ros::Publisher& pub);
void FpToRosMsg(const GP_HDT& data, ros::Publisher& pub);
void FpToRosMsg(const GP_RMC& data, ros::Publisher& pub);
void FpToRosMsg(const GP_VTG& data, ros::Publisher& pub);
void FpToRosMsg(const GP_ZDA& data, ros::Publisher& pub);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void TfDataToMsg(const TfData& data, geometry_msgs::TransformStamped& msg);

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
void OdometryDataToTf(const FP_ODOMETRY& data, tf2_ros::TransformBroadcaster& pub);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] tf
 */
void OdomToTf(const OdometryData& data, geometry_msgs::TransformStamped& tf);

/**
 * @brief
 *
 * @param[in] tf_map
 * @param[out] static_br_
 * @param[out] br_
 */
void PublishNav2Tf(const std::map<std::string, std::shared_ptr<geometry_msgs::TransformStamped>>& tf_map, tf2_ros::StaticTransformBroadcaster& static_br_, tf2_ros::TransformBroadcaster& br_);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdomToNavSatFix(const FP_ODOMETRY& data, ros::Publisher& pub);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdomToImuMsg(const FP_ODOMETRY& data, ros::Publisher& pub);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdomToYprMsg(const OdometryData& data, ros::Publisher& pub);

/**
 * @brief
 *
 * @param[in] stamp
 * @param[in] pos_diff
 * @param[in] prev_cov
 * @param[out] msg
 */
void JumpWarningMsg(const times::GpsTime& stamp, const Eigen::Vector3d& pos_diff, const Eigen::MatrixXd& prev_cov, ros::Publisher& pub);

}  // namespace fixposition

#endif
