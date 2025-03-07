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
 * @brief Convert data to ROS1 msgs
 */

#ifndef __FIXPOSITION_DRIVER_ROS1_DATA_TO_ROS1_HPP__
#define __FIXPOSITION_DRIVER_ROS1_DATA_TO_ROS1_HPP__

/* LIBC/STL */
#include <memory>
#include <unordered_map>

/* EXTERNAL */
#include <fixposition_driver_lib/fixposition_driver.hpp>
#include <fixposition_driver_lib/helper.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/novb.hpp>
#include <fpsdk_ros1/ext/ros.hpp>

/* PACKAGE */
#include "ros1_msgs.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

void TfDataToTransformStamped(const TfData& data, geometry_msgs::TransformStamped& msg);
void OdometryDataToTransformStamped(const OdometryData& data, geometry_msgs::TransformStamped& msg);

void PublishFpaOdometry(const fpsdk::common::parser::fpa::FpaOdometryPayload& payload, ros::Publisher& pub);
void PublishFpaOdometryDataImu(const fpsdk::common::parser::fpa::FpaOdometryPayload& payload, bool nav2_mode_, ros::Publisher& pub);
void PublishFpaOdometryDataNavSatFix(const fpsdk::common::parser::fpa::FpaOdometryPayload& payload, bool nav2_mode_, 
                                     ros::Publisher& pub);
void PublishFpaOdomenu(const fpsdk::common::parser::fpa::FpaOdomenuPayload& payload, ros::Publisher& pub);
void PublishFpaOdomenuVector3Stamped(const fpsdk::common::parser::fpa::FpaOdomenuPayload& payload, ros::Publisher& pub);
void PublishFpaOdomsh(const fpsdk::common::parser::fpa::FpaOdomshPayload& payload, ros::Publisher& pub);
void PublishFpaOdomstatus(const fpsdk::common::parser::fpa::FpaOdomstatusPayload& payload, ros::Publisher& pub);
void PublishFpaLlh(const fpsdk::common::parser::fpa::FpaLlhPayload& payload, ros::Publisher& pub);
void PublishFpaEoe(const fpsdk::common::parser::fpa::FpaEoePayload& payload, ros::Publisher& pub);
void PublishFpaImubias(const fpsdk::common::parser::fpa::FpaImubiasPayload& payload, ros::Publisher& pub);
void PublishFpaGnssant(const fpsdk::common::parser::fpa::FpaGnssantPayload& payload, ros::Publisher& pub);
void PublishFpaGnsscorr(const fpsdk::common::parser::fpa::FpaGnsscorrPayload& payload, ros::Publisher& pub);
void PublishFpaTp(const fpsdk::common::parser::fpa::FpaTpPayload& payload, ros::Publisher& pub);
void PublishFpaText(const fpsdk::common::parser::fpa::FpaTextPayload& payload, ros::Publisher& pub);
void PublishFpaRawimu(const fpsdk::common::parser::fpa::FpaRawimuPayload& payload, ros::Publisher& pub);
void PublishFpaCorrimu(const fpsdk::common::parser::fpa::FpaCorrimuPayload& payload, ros::Publisher& pub);

bool PublishNovbBestgnsspos(const fpsdk::common::parser::novb::NovbHeader* header,
                            const fpsdk::common::parser::novb::NovbBestgnsspos* payload, ros::Publisher& pub1,
                            ros::Publisher& pub2);
bool PublishNovbInspvax(const fpsdk::common::parser::novb::NovbHeader* header,
                        const fpsdk::common::parser::novb::NovbInspvax* payload, ros::Publisher& pub);

void PublishNmeaGga(const fpsdk::common::parser::nmea::NmeaGgaPayload& payload, ros::Publisher& pub);
void PublishNmeaGll(const fpsdk::common::parser::nmea::NmeaGllPayload& payload, ros::Publisher& pub);
void PublishNmeaGsa(const fpsdk::common::parser::nmea::NmeaGsaPayload& payload, ros::Publisher& pub);
void PublishNmeaGst(const fpsdk::common::parser::nmea::NmeaGstPayload& payload, ros::Publisher& pub);
void PublishNmeaGsv(const fpsdk::common::parser::nmea::NmeaGsvPayload& payload, ros::Publisher& pub);
void PublishNmeaHdt(const fpsdk::common::parser::nmea::NmeaHdtPayload& payload, ros::Publisher& pub);
void PublishNmeaRmc(const fpsdk::common::parser::nmea::NmeaRmcPayload& payload, ros::Publisher& pub);
void PublishNmeaVtg(const fpsdk::common::parser::nmea::NmeaVtgPayload& payload, ros::Publisher& pub);
void PublishNmeaZda(const fpsdk::common::parser::nmea::NmeaZdaPayload& payload, ros::Publisher& pub);

void PublishParserMsg(const fpsdk::common::parser::ParserMsg& msg, ros::Publisher& pub);

void PublishNmeaEpochData(const NmeaEpochData& data, ros::Publisher& pub);
void PublishOdometryData(const OdometryData& data, ros::Publisher& pub);
void PublishJumpWarning(const JumpDetector& jump_detector, ros::Publisher& pub);
void PublishDatum(const geometry_msgs::Vector3& payload, const ros::Time& stamp, ros::Publisher& pub);
void PublishFusionEpochData(const FusionEpochData& data, ros::Publisher& pub);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS1_DATA_TO_ROS1_HPP__
