/**
 *  @file
 *  @brief Convert Data classes to ROS2 msgs
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

#ifndef __FIXPOSITION_DRIVER_ROS2_DATA_TO_ROS2__
#define __FIXPOSITION_DRIVER_ROS2_DATA_TO_ROS2__

/* FIXPOSITION DRIVER LIB */
#include <fixposition_driver_lib/messages/msg_data.hpp>
#include <fixposition_driver_lib/fixposition_driver.hpp>

/* PACKAGE */
#include <fixposition_driver_ros2/fixposition_driver_node.hpp>

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
void FpToRosMsg( const OdometryData& data, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub);
void FpToRosMsg(      const ImuData& data, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub);
void FpToRosMsg(       const FP_EOE& data, rclcpp::Publisher<fixposition_driver_ros2::msg::EOE>::SharedPtr pub);
void FpToRosMsg(   const FP_GNSSANT& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GNSSANT>::SharedPtr pub);
void FpToRosMsg(  const FP_GNSSCORR& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GNSSCORR>::SharedPtr pub);
void FpToRosMsg(   const FP_IMUBIAS& data, rclcpp::Publisher<fixposition_driver_ros2::msg::IMUBIAS>::SharedPtr pub);
void FpToRosMsg(       const FP_LLH& data, rclcpp::Publisher<fixposition_driver_ros2::msg::LLH>::SharedPtr pub);
void FpToRosMsg(   const FP_ODOMENU& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMENU>::SharedPtr pub);
void FpToRosMsg(  const FP_ODOMETRY& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMETRY>::SharedPtr pub);
void FpToRosMsg(    const FP_ODOMSH& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMSH>::SharedPtr pub);
void FpToRosMsg(const FP_ODOMSTATUS& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMSTATUS>::SharedPtr pub);
void FpToRosMsg(        const FP_TP& data, rclcpp::Publisher<fixposition_driver_ros2::msg::TP>::SharedPtr pub);
void FpToRosMsg(      const FP_TEXT& data, rclcpp::Publisher<fixposition_driver_ros2::msg::TEXT>::SharedPtr pub);

void FpToRosMsg(const GP_GGA& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPGGA>::SharedPtr pub);
void FpToRosMsg(const GP_GLL& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPGLL>::SharedPtr pub);
void FpToRosMsg(const GN_GSA& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GNGSA>::SharedPtr pub);
void FpToRosMsg(const GP_GST& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPGST>::SharedPtr pub);
void FpToRosMsg(const GX_GSV& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GXGSV>::SharedPtr pub);
void FpToRosMsg(const GP_HDT& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPHDT>::SharedPtr pub);
void FpToRosMsg(const GP_RMC& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPRMC>::SharedPtr pub);
void FpToRosMsg(const GP_VTG& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPVTG>::SharedPtr pub);
void FpToRosMsg(const GP_ZDA& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPZDA>::SharedPtr pub);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void TfDataToMsg(const TfData& data, geometry_msgs::msg::TransformStamped& msg);

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
void OdometryDataToTf(const FP_ODOMETRY& data, std::shared_ptr<tf2_ros::TransformBroadcaster> pub);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] tf
 */
void OdomToTf(const OdometryData& data, geometry_msgs::msg::TransformStamped& tf);

/**
 * @brief
 *
 * @param[in] tf_map
 * @param[out] static_br_
 * @param[out] br_
 */
void PublishNav2Tf(const std::map<std::string, std::shared_ptr<geometry_msgs::msg::TransformStamped>>& tf_map, std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_, std::shared_ptr<tf2_ros::TransformBroadcaster> br_);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdomToNavSatFix(const fixposition::FP_ODOMETRY& data, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdomToImuMsg(const fixposition::FP_ODOMETRY& data, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr msg);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdomToYprMsg(const OdometryData& data, rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub);

/**
 * @brief
 *
 * @param[in] stamp
 * @param[in] pos_diff
 * @param[in] prev_cov
 * @param[out] msg
 */
void JumpWarningMsg(std::shared_ptr<rclcpp::Node> node, const times::GpsTime& stamp, const Eigen::Vector3d& pos_diff, 
                    const Eigen::MatrixXd& prev_cov, rclcpp::Publisher<fixposition_driver_ros2::msg::COVWARN>::SharedPtr pub);

}  // namespace fixposition

#endif
