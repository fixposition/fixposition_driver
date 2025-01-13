/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Convert data to ROS2 msgs
 */

#ifndef __FIXPOSITION_DRIVER_ROS2_DATA_TO_ROS2_HPP__
#define __FIXPOSITION_DRIVER_ROS2_DATA_TO_ROS2_HPP__

/* LIBC/STL */
#include <memory>
#include <unordered_map>

/* EXTERNAL */
#include <fixposition_driver_lib/fixposition_driver.hpp>
#include <fixposition_driver_lib/helper.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/novb.hpp>
#include <fpsdk_ros2/ext/rclcpp.hpp>

/* PACKAGE */
#include "ros2_msgs.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

void TfDataToTransformStamped(const TfData& data, geometry_msgs::msg::TransformStamped& msg);
void OdometryDataToTransformStamped(const OdometryData& data, geometry_msgs::msg::TransformStamped& msg);

void PublishFpaOdometry(const fpsdk::common::parser::fpa::FpaOdometryPayload& payload,
                        rclcpp::Publisher<fpmsgs::FpaOdometry>::SharedPtr& pub);
void PublishFpaOdometryDataImu(const fpsdk::common::parser::fpa::FpaOdometryPayload& payload,
                               rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& pub);
void PublishFpaOdometryDataNavSatFix(const fpsdk::common::parser::fpa::FpaOdometryPayload& payload,
                                     rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& pub);
void PublishFpaOdomenu(const fpsdk::common::parser::fpa::FpaOdomenuPayload& payload,
                       rclcpp::Publisher<fpmsgs::FpaOdomenu>::SharedPtr& pub);
void PublishFpaOdomenuVector3Stamped(const fpsdk::common::parser::fpa::FpaOdomenuPayload& payload,
                                     rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr& pub);
void PublishFpaOdomsh(const fpsdk::common::parser::fpa::FpaOdomshPayload& payload,
                      rclcpp::Publisher<fpmsgs::FpaOdomsh>::SharedPtr& pub);
void PublishFpaOdomstatus(const fpsdk::common::parser::fpa::FpaOdomstatusPayload& payload,
                          rclcpp::Publisher<fpmsgs::FpaOdomstatus>::SharedPtr& pub);
void PublishFpaLlh(const fpsdk::common::parser::fpa::FpaLlhPayload& payload,
                   rclcpp::Publisher<fpmsgs::FpaLlh>::SharedPtr& pub);
void PublishFpaEoe(const fpsdk::common::parser::fpa::FpaEoePayload& payload,
                   rclcpp::Publisher<fpmsgs::FpaEoe>::SharedPtr& pub);
void PublishFpaImubias(const fpsdk::common::parser::fpa::FpaImubiasPayload& payload,
                       rclcpp::Publisher<fpmsgs::FpaImubias>::SharedPtr& pub);
void PublishFpaGnssant(const fpsdk::common::parser::fpa::FpaGnssantPayload& payload,
                       rclcpp::Publisher<fpmsgs::FpaGnssant>::SharedPtr& pub);
void PublishFpaGnsscorr(const fpsdk::common::parser::fpa::FpaGnsscorrPayload& payload,
                        rclcpp::Publisher<fpmsgs::FpaGnsscorr>::SharedPtr& pub);
void PublishFpaTp(const fpsdk::common::parser::fpa::FpaTpPayload& payload,
                  rclcpp::Publisher<fpmsgs::FpaTp>::SharedPtr& pub);
void PublishFpaText(const fpsdk::common::parser::fpa::FpaTextPayload& payload,
                    rclcpp::Publisher<fpmsgs::FpaText>::SharedPtr& pub);
void PublishFpaRawimu(const fpsdk::common::parser::fpa::FpaRawimuPayload& payload,
                      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& pub);
void PublishFpaCorrimu(const fpsdk::common::parser::fpa::FpaCorrimuPayload& payload,
                       rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& pub);

bool PublishNovbBestgnsspos(const fpsdk::common::parser::novb::NovbHeader* header,
                            const fpsdk::common::parser::novb::NovbBestgnsspos* payload,
                            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& pub1,
                            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& pub2);

void PublishNmeaGga(const fpsdk::common::parser::nmea::NmeaGgaPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGga>::SharedPtr& pub);
void PublishNmeaGll(const fpsdk::common::parser::nmea::NmeaGllPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGll>::SharedPtr& pub);
void PublishNmeaGsa(const fpsdk::common::parser::nmea::NmeaGsaPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGsa>::SharedPtr& pub);
void PublishNmeaGst(const fpsdk::common::parser::nmea::NmeaGstPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGst>::SharedPtr& pub);
void PublishNmeaGsv(const fpsdk::common::parser::nmea::NmeaGsvPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGsv>::SharedPtr& pub);
void PublishNmeaHdt(const fpsdk::common::parser::nmea::NmeaHdtPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaHdt>::SharedPtr& pub);
void PublishNmeaRmc(const fpsdk::common::parser::nmea::NmeaRmcPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaRmc>::SharedPtr& pub);
void PublishNmeaVtg(const fpsdk::common::parser::nmea::NmeaVtgPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaVtg>::SharedPtr& pub);
void PublishNmeaZda(const fpsdk::common::parser::nmea::NmeaZdaPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaZda>::SharedPtr& pub);

void PublishParserMsg(const fpsdk::common::parser::ParserMsg& msg,
                      rclcpp::Publisher<fpmsgs::ParserMsg>::SharedPtr& pub);

void PublishNmeaEpochData(const NmeaEpochData& data, rclcpp::Publisher<fpmsgs::NmeaEpoch>::SharedPtr& pub);
void PublishOdometryData(const OdometryData& data, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pub);
void PublishJumpWarning(const JumpDetector& jump_detector, rclcpp::Publisher<fpmsgs::CovWarn>::SharedPtr& pub);

#if 0
inline void PublishFpaOdometry(const fpsdk::common::parser::fpa::FpaOdometryPayload& payload,
                               rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMETRY>::SharedPtr& pub) {
    (void)payload;
    if (pub->get_subscription_count() > 0) {
        fixposition_driver_ros2::msg::ODOMETRY msg;
        msg.header.stamp = rclcpp::Clock().now();
        pub->publish(msg);
    }
}

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
void FpToRosMsg(const OdometryData& data, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub);
void FpToRosMsg(const ImuData& data, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub);
void FpToRosMsg(const FP_EOE& data, rclcpp::Publisher<fixposition_driver_ros2::msg::EOE>::SharedPtr pub);
void FpToRosMsg(const FP_GNSSANT& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GNSSANT>::SharedPtr pub);
void FpToRosMsg(const FP_GNSSCORR& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GNSSCORR>::SharedPtr pub);
void FpToRosMsg(const FP_IMUBIAS& data, rclcpp::Publisher<fixposition_driver_ros2::msg::IMUBIAS>::SharedPtr pub);
void FpToRosMsg(const FP_LLH& data, rclcpp::Publisher<fixposition_driver_ros2::msg::LLH>::SharedPtr pub);
void FpToRosMsg(const FP_ODOMENU& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMENU>::SharedPtr pub);
void FpToRosMsg(const FP_ODOMETRY& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMETRY>::SharedPtr pub);
void FpToRosMsg(const FP_ODOMSH& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMSH>::SharedPtr pub);
void FpToRosMsg(const FP_ODOMSTATUS& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMSTATUS>::SharedPtr pub);
void FpToRosMsg(const FP_TP& data, rclcpp::Publisher<fixposition_driver_ros2::msg::TP>::SharedPtr pub);
void FpToRosMsg(const FP_TEXT& data, rclcpp::Publisher<fixposition_driver_ros2::msg::TEXT>::SharedPtr pub);

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
void PublishNav2Tf(const std::map<std::string, std::shared_ptr<geometry_msgs::msg::TransformStamped>>& tf_map,
                   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_,
                   std::shared_ptr<tf2_ros::TransformBroadcaster> br_);

/**
 * @brief
 *
 * @param[in] data
 * @param[out] msg
 */
void OdomToNavSatFix(const fixposition::FP_ODOMETRY& data,
                     rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub);

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
                    const Eigen::MatrixXd& prev_cov,
                    rclcpp::Publisher<fixposition_driver_ros2::msg::COVWARN>::SharedPtr pub);

#endif

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS2_DATA_TO_ROS2_HPP__
