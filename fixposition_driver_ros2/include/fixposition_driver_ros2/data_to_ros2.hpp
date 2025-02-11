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
bool PublishNovbInspvax(const fpsdk::common::parser::novb::NovbHeader* header,
                        const fpsdk::common::parser::novb::NovbInspvax* payload,
                        rclcpp::Publisher<fpmsgs::NovbInspvax>::SharedPtr& pub);

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
void PublishFusionEpochData(const FusionEpochData& data, rclcpp::Publisher<fpmsgs::FusionEpoch>::SharedPtr& pub);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS2_DATA_TO_ROS2_HPP__
