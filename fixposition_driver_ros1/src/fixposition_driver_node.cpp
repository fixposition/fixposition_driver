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
 * @brief Main function for the fixposition driver ros node
 */

/* LIBC/STL */
#include <cstring>
#include <functional>
#include <memory>

/* EXTERNAL */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/trafo.hpp>
#include <fpsdk_common/types.hpp>
#include <fpsdk_ros1/utils.hpp>

/* PACKAGE */
#include "fixposition_driver_ros1/fixposition_driver_node.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::common;
using namespace fpsdk::common::parser;

FixpositionDriverNode::FixpositionDriverNode(const SensorParams& sensor_params, const NodeParams& node_params,
                                             ros::NodeHandle& nh) /* clang-format off */ :
    nh_              { nh },
    sensor_params_   { sensor_params },
    node_params_     { node_params },
    driver_          { sensor_params }  // clang-format on
{}

FixpositionDriverNode::~FixpositionDriverNode() { StopNode(); }

// ---------------------------------------------------------------------------------------------------------------------

// Helper for subscribing to input topics
#define _SUB(_handle_, _msg_type_, _topic_, ...)         \
    ROS_INFO("Subscribe %s (" #_msg_type_ ")", _topic_); \
    _handle_ = nh_.subscribe<_msg_type_>(_topic_, __VA_ARGS__)

// Helper for advertising output topics
#define _PUB(_pub_, _msg_type_, _topic_, ...)                    \
    if (_pub_.getTopic().empty()) {                              \
        ROS_INFO("Advertise %s (" #_msg_type_ ")", _topic_);     \
        _pub_ = nh_.advertise<_msg_type_>(_topic_, __VA_ARGS__); \
    }

bool FixpositionDriverNode::StartNode() {
    ROS_INFO("Starting...");

    // const auto hints = ros::TransportHints().tcpNoDelay(true);
    // _SUB(sub_fusion_odom_status_,   fixposition_msgs::FusionOdomStatus,   t.odom_status_topic_,     25,
    // &UserIoCore::ProcessFusionOdomStatus,   iocore_.get(), hints);

    // Add observers and advertise output topics, depending on configuration

    // NOV_B-BESTGNSSPOS
    if (sensor_params_.MessageEnabled(novb::NOV_B_BESTGNSSPOS_STRID)) {
        _PUB(navsatfix_gnss1_pub_, sensor_msgs::NavSatFix, "/fixposition/gnss1", 5);
        _PUB(navsatfix_gnss2_pub_, sensor_msgs::NavSatFix, "/fixposition/gnss2", 5);
        driver_.AddNovbObserver(novb::NOV_B_BESTGNSSPOS_STRID,
                                std::bind(&FixpositionDriverNode::HandleNovbMessage, this, std::placeholders::_1));
    }

    // // Advertise output topics
    // // - FP_A messages
    // fpa_odometry_pub_ = nh_.advertise<fixposition_driver_ros1::odometry>("/fixposition/fpa/odometry", 5);
    // fpa_imubias_pub_ = nh_.advertise<fixposition_driver_ros1::imubias>("/fixposition/fpa/imubias", 5);
    // fpa_eoe_pub_ = nh_.advertise<fixposition_driver_ros1::eoe>("/fixposition/fpa/eoe", 5);
    // fpa_llh_pub_ = nh_.advertise<fixposition_driver_ros1::llh>("/fixposition/fpa/llh", 5);
    // fpa_odomenu_pub_ = nh_.advertise<fixposition_driver_ros1::odomenu>("/fixposition/fpa/odomenu", 5);
    // fpa_odomsh_pub_ = nh_.advertise<fixposition_driver_ros1::odomsh>("/fixposition/fpa/odomsh", 5);
    // fpa_odomstatus_pub_ = nh_.advertise<fixposition_driver_ros1::odomstatus>("/fixposition/fpa/odomstatus", 5);
    // fpa_gnssant_pub_ = nh_.advertise<fixposition_driver_ros1::gnssant>("/fixposition/fpa/gnssant", 5);
    // fpa_gnsscorr_pub_ = nh_.advertise<fixposition_driver_ros1::gnsscorr>("/fixposition/fpa/gnsscorr", 5);
    // fpa_text_pub_ = nh_.advertise<fixposition_driver_ros1::text>("/fixposition/fpa/text", 5);
    // fpa_tp_pub_ = nh_.advertise<fixposition_driver_ros1::tp>("/fixposition/fpa/tp", 5);
    // // - NMEA messages
    // nmea_gpgga_pub_ = nh_.advertise<fixposition_driver_ros1::gpgga>("/fixposition/nmea/gpgga", 5);
    // nmea_gpgll_pub_ = nh_.advertise<fixposition_driver_ros1::gpgll>("/fixposition/nmea/gpgll", 5);
    // nmea_gngsa_pub_ = nh_.advertise<fixposition_driver_ros1::gngsa>("/fixposition/nmea/gngsa", 5);
    // nmea_gpgst_pub_ = nh_.advertise<fixposition_driver_ros1::gpgst>("/fixposition/nmea/gpgst", 5);
    // nmea_gxgsv_pub_ = nh_.advertise<fixposition_driver_ros1::gxgsv>("/fixposition/nmea/gxgsv", 5);
    // nmea_gphdt_pub_ = nh_.advertise<fixposition_driver_ros1::gphdt>("/fixposition/nmea/gphdt", 5);
    // nmea_gprmc_pub_ = nh_.advertise<fixposition_driver_ros1::gprmc>("/fixposition/nmea/gprmc", 5);
    // nmea_gpvtg_pub_ = nh_.advertise<fixposition_driver_ros1::gpvtg>("/fixposition/nmea/gpvtg", 5);
    // nmea_gpzda_pub_ = nh_.advertise<fixposition_driver_ros1::gpzda>("/fixposition/nmea/gpzda", 5);
    // // - Odometry
    // odometry_ecef_pub_ = nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_ecef", 5);
    // odometry_llh_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/odometry_llh", 5);
    // odometry_enu_pub_ = nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_enu", 5);
    // odometry_smooth_pub_ = nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_smooth", 5);
    // // - Orientation
    // eul_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/fixposition/ypr", 5);
    // eul_imu_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/fixposition/imu_ypr", 5);
    // // - IMU
    // rawimu_pub_ = nh_.advertise<sensor_msgs::Imu>("/fixposition/rawimu", 5);
    // corrimu_pub_ = nh_.advertise<sensor_msgs::Imu>("/fixposition/corrimu", 5);
    // poiimu_pub_ = nh_.advertise<sensor_msgs::Imu>("/fixposition/poiimu", 5);
    // // - GNSS
    // nmea_pub_ = nh_.advertise<fixposition_driver_ros1::NMEA>("/fixposition/nmea", 5);

    // // Subscribe to input topics
    // ws_sub_ = nh_.subscribe<fixposition_driver_ros1::Speed>(node_params_.speed_topic_, 10,
    //                                                         &FixpositionDriverNode::WsCallbackRos, this);
    // rtcm_sub_ =
    //     nh_.subscribe<rtcm_msgs::Message>(node_params_.corr_topic_, 10, &FixpositionDriverNode::RtcmCallbackRos,
    //     this);

    // // Configure jump warning message
    // if (sensor_params_.cov_warning_) {
    //     extras_jump_pub_ = nh_.advertise<fixposition_driver_ros1::CovWarn>("/fixposition/extras/jump", 5);
    // }
    // prev_pos_.setZero();
    // prev_cov_.setZero();

    // AddFpaObserver(fpa::FpaEoePayload::MSG_NAME, [](const fpa::FpaPayload& payload) {
    //     auto eoe = dynamic_cast<const fpa::FpaEoePayload&>(payload);
    //     ROS_WARN("EOE --> %.1f %d", eoe.gps_time.tow.value, types::EnumToVal(eoe.epoch));
    // });

    // AddFpaObserver(fpa::FpaOdometryPayload::MSG_NAME, [](const fpa::FpaPayload& payload) {
    //     auto odo = dynamic_cast<const fpa::FpaOdometryPayload&>(payload);
    //     ROS_WARN("ODOMETRY --> %.1f %d", odo.gps_time.tow.value, types::EnumToVal(odo.which));
    // });

    // AddNmeaObserver(nmea::NmeaHdtPayload::FORMATTER, [](const nmea::NmeaPayload& payload) {
    //     auto hdt = dynamic_cast<const nmea::NmeaHdtPayload&>(payload);
    //     ROS_WARN("HDT --> %.1f", hdt.heading.value);

    // });

    // AddNovbObserver(novb::NOV_B_BESTGNSSPOS_STRID, [](const ParserMsg& msg) {
    //     ROS_WARN("NOVB --> %s", msg.name_.c_str());
    // });

    return driver_.StartDriver();
}

#undef _PUB
#undef _SUB

void FixpositionDriverNode::StopNode() {
    ROS_INFO("Stopping...");

    driver_.RemoveFpaObservers();
    driver_.RemoveNmeaObservers();
    driver_.RemoveNovbObservers();

    driver_.StopDriver();

    // Stop advertising output topics
    // // - FP_A messages
    // fpa_odometry_pub_.shutdown();
    // fpa_imubias_pub_.shutdown();
    // fpa_eoe_pub_.shutdown();
    // fpa_llh_pub_.shutdown();
    // fpa_odomenu_pub_.shutdown();
    // fpa_odomsh_pub_.shutdown();
    // fpa_odomstatus_pub_.shutdown();
    // fpa_gnssant_pub_.shutdown();
    // fpa_gnsscorr_pub_.shutdown();
    // fpa_text_pub_.shutdown();
    // fpa_tp_pub_.shutdown();
    // // - NMEA messages
    // nmea_gpgga_pub_.shutdown();
    // nmea_gpgll_pub_.shutdown();
    // nmea_gngsa_pub_.shutdown();
    // nmea_gpgst_pub_.shutdown();
    // nmea_gxgsv_pub_.shutdown();
    // nmea_gphdt_pub_.shutdown();
    // nmea_gprmc_pub_.shutdown();
    // nmea_gpvtg_pub_.shutdown();
    // nmea_gpzda_pub_.shutdown();
    // // - Odometry
    // odometry_ecef_pub_.shutdown();
    // odometry_llh_pub_.shutdown();
    // odometry_enu_pub_.shutdown();
    // odometry_smooth_pub_.shutdown();
    // // - Orientation
    // eul_pub_.shutdown();
    // eul_imu_pub_.shutdown();
    // // - IMU
    // rawimu_pub_.shutdown();
    // corrimu_pub_.shutdown();
    // poiimu_pub_.shutdown();
    // // - GNSS
    // nmea_pub_.shutdown();
    navsatfix_gnss1_pub_.shutdown();
    navsatfix_gnss2_pub_.shutdown();

    // Stop input message subscribers
    // ws_sub_.shutdown();
    // rtcm_sub_.shutdown();
    // extras_jump_pub_.shutdown();
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::HandleNovbMessage(const fpsdk::common::parser::ParserMsg& msg) {
    TRACE("HandleNovbMessage() %s", msg.name_.c_str());

    const uint8_t* frame = msg.data_.data();
    const uint16_t msg_id = novb::NovbMsgId(frame);
    const bool is_long_header = novb::NovbIsLongHeader(frame);

    // NOV_B-BESTGNSSPOS
    if ((msg_id == novb::NOV_B_BESTGNSSPOS_MSGID) && is_long_header) {
        // Get message header and payload
        novb::NovbLongHeader header;
        novb::NovbBestgnsspos payload;
        std::memcpy(&header, &frame[0], sizeof(header));
        std::memcpy(&payload, &frame[sizeof(header)], sizeof(payload));

        // Convert to data
        NavSatFixData nav_sat_fix;
        BestGnssPosToNavSatFix(header, payload, nav_sat_fix);

        // Publish
        if (nav_sat_fix.frame_id == "GNSS1") {
            if (navsatfix_gnss1_pub_.getNumSubscribers() > 0) {
                sensor_msgs::NavSatFix ros_msg;
                NavSatFixDataToMsg(nav_sat_fix, ros_msg);
                navsatfix_gnss1_pub_.publish(ros_msg);
            }
        } else if (nav_sat_fix.frame_id == "GNSS2") {
            if (navsatfix_gnss2_pub_.getNumSubscribers() > 0) {
                sensor_msgs::NavSatFix ros_msg;
                NavSatFixDataToMsg(nav_sat_fix, ros_msg);
                navsatfix_gnss2_pub_.publish(ros_msg);
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "Bad %s (frame_id %s)", msg.name_.c_str(), nav_sat_fix.frame_id.c_str());
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

#if 0
void FixpositionDriverNode::RegisterObservers() {
    // NOV_B

    // FP_A
    for (const auto& format : sensor_params_.formats) {
        if (format == "ODOMETRY") {
            dynamic_cast<NmeaConverter<FP_ODOMETRY>*>(a_converters_["ODOMETRY"].get())
                ->AddObserver([this](const FP_ODOMETRY& data) {
                    FpToRosMsg(data, fpa_odometry_pub_);
                    FpToRosMsg(data.odom, odometry_ecef_pub_);
                    OdomToImuMsg(data, poiimu_pub_);
                    OdomToNavSatFix(data, odometry_llh_pub_);
                    OdometryDataToTf(data, br_);

                    // Output jump warning
                    if (sensor_params_.cov_warning_) {
                        if (!prev_pos_.isZero() && !prev_cov_.isZero()) {
                            Eigen::Vector3d pos_diff = (prev_pos_ - data.odom.pose.position).cwiseAbs();

                            if ((pos_diff[0] > prev_cov_(0, 0)) || (pos_diff[1] > prev_cov_(1, 1)) ||
                                (pos_diff[2] > prev_cov_(2, 2))) {
                                JumpWarningMsg(data.odom.stamp, pos_diff, prev_cov_, extras_jump_pub_);
                            }
                        }
                        prev_pos_ = data.odom.pose.position;
                        prev_cov_ = data.odom.pose.cov;
                    }
                });
        } else if (format == "ODOMENU") {
            dynamic_cast<NmeaConverter<FP_ODOMENU>*>(a_converters_["ODOMENU"].get())
                ->AddObserver([this](const FP_ODOMENU& data) {
                    FpToRosMsg(data, fpa_odomenu_pub_);
                    FpToRosMsg(data.odom, odometry_enu_pub_);
                    OdomToYprMsg(data.odom, eul_pub_);

                    // Append TF if Nav2 mode is selected
                    if (sensor_params_.nav2_mode_) {
                        // Get FP_ENU0 -> FP_POI
                        geometry_msgs::TransformStamped tf;
                        OdomToTf(data.odom, tf);
                        tf_map["ENU0POI"] = std::make_shared<geometry_msgs::TransformStamped>(tf);
                    }
                });
        } else if (format == "ODOMSH") {
            dynamic_cast<NmeaConverter<FP_ODOMSH>*>(a_converters_["ODOMSH"].get())
                ->AddObserver([this](const FP_ODOMSH& data) {
                    FpToRosMsg(data, fpa_odomsh_pub_);
                    FpToRosMsg(data.odom, odometry_smooth_pub_);

                    // Append TF if Nav2 mode is selected
                    if (sensor_params_.nav2_mode_) {
                        // Get FP_ECEF -> FP_POISH
                        geometry_msgs::TransformStamped tf;
                        OdomToTf(data.odom, tf);
                        tf_map["ECEFPOISH"] = std::make_shared<geometry_msgs::TransformStamped>(tf);
                    }
                });
        } else if (format == "ODOMSTATUS") {
            dynamic_cast<NmeaConverter<FP_ODOMSTATUS>*>(a_converters_["ODOMSTATUS"].get())
                ->AddObserver([this](const FP_ODOMSTATUS& data) { FpToRosMsg(data, fpa_odomstatus_pub_); });
        } else if (format == "IMUBIAS") {
            dynamic_cast<NmeaConverter<FP_IMUBIAS>*>(a_converters_["IMUBIAS"].get())
                ->AddObserver([this](const FP_IMUBIAS& data) { FpToRosMsg(data, fpa_imubias_pub_); });
        } else if (format == "EOE") {
            dynamic_cast<NmeaConverter<FP_EOE>*>(a_converters_["EOE"].get())->AddObserver([this](const FP_EOE& data) {
                FpToRosMsg(data, fpa_eoe_pub_);

                // Generate Nav2 TF tree
                if (data.epoch == "FUSION" && sensor_params_.nav2_mode_) {
                    PublishNav2Tf(tf_map, static_br_, br_);
                }
            });
        } else if (format == "LLH") {
            dynamic_cast<NmeaConverter<FP_LLH>*>(a_converters_["LLH"].get())->AddObserver([this](const FP_LLH& data) {
                FpToRosMsg(data, fpa_llh_pub_);
            });
        } else if (format == "GNSSANT") {
            dynamic_cast<NmeaConverter<FP_GNSSANT>*>(a_converters_["GNSSANT"].get())
                ->AddObserver([this](const FP_GNSSANT& data) { FpToRosMsg(data, fpa_gnssant_pub_); });
        } else if (format == "GNSSCORR") {
            dynamic_cast<NmeaConverter<FP_GNSSCORR>*>(a_converters_["GNSSCORR"].get())
                ->AddObserver([this](const FP_GNSSCORR& data) { FpToRosMsg(data, fpa_gnsscorr_pub_); });
        } else if (format == "TEXT") {
            dynamic_cast<NmeaConverter<FP_TEXT>*>(a_converters_["TEXT"].get())
                ->AddObserver([this](const FP_TEXT& data) { FpToRosMsg(data, fpa_text_pub_); });
        } else if (format == "RAWIMU") {
            dynamic_cast<NmeaConverter<FP_RAWIMU>*>(a_converters_["RAWIMU"].get())
                ->AddObserver([this](const FP_RAWIMU& data) { FpToRosMsg(data.imu, rawimu_pub_); });
        } else if (format == "CORRIMU") {
            dynamic_cast<NmeaConverter<FP_CORRIMU>*>(a_converters_["CORRIMU"].get())
                ->AddObserver([this](const FP_CORRIMU& data) { FpToRosMsg(data.imu, corrimu_pub_); });
        } else if (format == "TF") {
            dynamic_cast<NmeaConverter<FP_TF>*>(a_converters_["TF"].get())->AddObserver([this](const FP_TF& data) {
                if (data.valid_tf) {
                    // TF Observer Lambda
                    geometry_msgs::TransformStamped tf;
                    TfDataToMsg(data.tf, tf);
                    if (tf.child_frame_id == "FP_IMUH" && tf.header.frame_id == "FP_POI") {
                        br_.sendTransform(tf);

                        // Publish Pitch Roll based on IMU only
                        Eigen::Vector3d imu_ypr_eigen = trafo::QuatToEul(data.tf.rotation);
                        imu_ypr_eigen.x() = 0.0;  // the yaw value is not observable using IMU alone
                        geometry_msgs::Vector3Stamped imu_ypr;
                        imu_ypr.header.stamp = tf.header.stamp;
                        imu_ypr.header.frame_id = "FP_POI";
                        tf::vectorEigenToMsg(imu_ypr_eigen, imu_ypr.vector);
                        eul_imu_pub_.publish(imu_ypr);

                    } else if (tf.child_frame_id == "FP_POISH" && tf.header.frame_id == "FP_POI") {
                        br_.sendTransform(tf);

                        // Append TF if Nav2 mode is selected
                        if (sensor_params_.nav2_mode_) {
                            // Get FP_POI -> FP_POISH
                            tf_map["POIPOISH"] = std::make_shared<geometry_msgs::TransformStamped>(tf);
                        }
                    } else if (tf.child_frame_id == "FP_ENU0" && tf.header.frame_id == "FP_ECEF") {
                        static_br_.sendTransform(tf);

                        // Append TF if Nav2 mode is selected
                        if (sensor_params_.nav2_mode_) {
                            // Get FP_ECEF -> FP_ENU0
                            tf_map["ECEFENU0"] = std::make_shared<geometry_msgs::TransformStamped>(tf);
                        }
                    } else {
                        static_br_.sendTransform(tf);
                    }
                }
            });
        } else if (format == "TP") {
            dynamic_cast<NmeaConverter<FP_TP>*>(a_converters_["TP"].get())->AddObserver([this](const FP_TP& data) {
                FpToRosMsg(data, fpa_tp_pub_);
            });
        } else if (format == "GPGGA") {
            dynamic_cast<NmeaConverter<GP_GGA>*>(a_converters_["GPGGA"].get())->AddObserver([this](const GP_GGA& data) {
                FpToRosMsg(data, nmea_gpgga_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                    PublishNmea();  // Receiving a GPGGA triggers the NMEA output... :-/
                }
            });
        } else if (format == "GPGLL") {
            dynamic_cast<NmeaConverter<GP_GLL>*>(a_converters_["GPGLL"].get())->AddObserver([this](const GP_GLL& data) {
                FpToRosMsg(data, nmea_gpgll_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                }
            });
        } else if (format == "GNGSA") {
            dynamic_cast<NmeaConverter<GN_GSA>*>(a_converters_["GNGSA"].get())->AddObserver([this](const GN_GSA& data) {
                FpToRosMsg(data, nmea_gngsa_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                }
            });
        } else if (format == "GPGST") {
            dynamic_cast<NmeaConverter<GP_GST>*>(a_converters_["GPGST"].get())->AddObserver([this](const GP_GST& data) {
                FpToRosMsg(data, nmea_gpgst_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                }
            });
        } else if (format == "GXGSV") {
            dynamic_cast<NmeaConverter<GX_GSV>*>(a_converters_["GXGSV"].get())->AddObserver([this](const GX_GSV& data) {
                FpToRosMsg(data, nmea_gxgsv_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                }
            });
        } else if (format == "GPHDT") {
            dynamic_cast<NmeaConverter<GP_HDT>*>(a_converters_["GPHDT"].get())->AddObserver([this](const GP_HDT& data) {
                FpToRosMsg(data, nmea_gphdt_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                }
            });
        } else if (format == "GPRMC") {
            dynamic_cast<NmeaConverter<GP_RMC>*>(a_converters_["GPRMC"].get())->AddObserver([this](const GP_RMC& data) {
                FpToRosMsg(data, nmea_gprmc_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                }
            });
        } else if (format == "GPVTG") {
            dynamic_cast<NmeaConverter<GP_VTG>*>(a_converters_["GPVTG"].get())->AddObserver([this](const GP_VTG& data) {
                FpToRosMsg(data, nmea_gpvtg_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                }
            });
        } else if (format == "GPZDA") {
            dynamic_cast<NmeaConverter<GP_ZDA>*>(a_converters_["GPZDA"].get())->AddObserver([this](const GP_ZDA& data) {
                FpToRosMsg(data, nmea_gpzda_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                }
            });
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::PublishNmea() {
    // If epoch message is complete, generate NMEA output
    if (nmea_message_.checkEpoch()) {
        // Generate new message
        fixposition_driver_ros1::NMEA msg;

        // ROS Header
        if (nmea_message_.stamp.tow == 0.0 && nmea_message_.stamp.wno == 0) {
            msg.header.stamp = ros::Time::now();
        } else {
            msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(nmea_message_.stamp));
        }
        msg.header.frame_id = "FP_POI";

        // Time and date fields
        msg.time = nmea_message_.time_str;
        msg.date = nmea_message_.date_str;

        // Latitude [degrees]. Positive is north of equator; negative is south
        msg.latitude = nmea_message_.llh(0);

        // Longitude [degrees]. Positive is east of prime meridian; negative is west
        msg.longitude = nmea_message_.llh(1);

        // Altitude [m]. Positive is above the WGS-84 ellipsoid
        msg.altitude = nmea_message_.llh(2);

        // Quality indicator
        msg.quality = nmea_message_.quality;

        // Number of satellites
        msg.num_sv = nmea_message_.num_sv;

        // ID numbers of satellites used in solution
        for (unsigned int i = 0; i < nmea_message_.ids.size(); i++) {
            msg.ids.push_back(nmea_message_.ids.at(i));
        }

        // Dilution of precision
        msg.hdop_rec = nmea_message_.hdop_receiver;
        msg.pdop = nmea_message_.pdop;
        msg.hdop = nmea_message_.hdop;
        msg.vdop = nmea_message_.vdop;

        // Populate GNSS pseudorange error statistics
        msg.rms_range = nmea_message_.rms_range;
        msg.std_major = nmea_message_.std_major;
        msg.std_minor = nmea_message_.std_minor;
        msg.angle_major = nmea_message_.angle_major;
        msg.std_lat = nmea_message_.std_lat;
        msg.std_lon = nmea_message_.std_lon;
        msg.std_alt = nmea_message_.std_alt;

        // Position covariance [m^2]
        Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map =
            Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.covariance.data());
        cov_map = nmea_message_.cov;

        // Method employed to estimate covariance
        msg.cov_type = nmea_message_.cov_type;

        // Populate GNSS satellites in view
        for (auto gsv_it = nmea_message_.gnss_signals.begin(); gsv_it != nmea_message_.gnss_signals.end(); ++gsv_it) {
            SignalType msg_type = gsv_it->first;
            std::map<unsigned int, GnssSignalStats>* gnss_data = &gsv_it->second;

            // Populate GnssSats message
            fixposition_driver_ros1::GnssSats sats_msg;

            // Get constellation name
            if (msg_type == SignalType::GPS) {
                sats_msg.constellation = "GPS";
            } else if (msg_type == SignalType::Galileo) {
                sats_msg.constellation = "Galileo";
            } else if (msg_type == SignalType::BeiDou) {
                sats_msg.constellation = "BeiDou";
            } else if (msg_type == SignalType::GLONASS) {
                sats_msg.constellation = "GLONASS";
            } else {
                sats_msg.constellation = "Unknown";
            }

            // Get signal statistics
            for (auto it = gnss_data->begin(); it != gnss_data->end(); ++it) {
                unsigned int sat_id = it->first;
                GnssSignalStats signals = it->second;

                sats_msg.sat_id.push_back(sat_id);
                sats_msg.azim.push_back(signals.azim);
                sats_msg.elev.push_back(signals.elev);
                sats_msg.cno_l1.push_back(signals.cno_l1);
                sats_msg.cno_l2.push_back(signals.cno_l2);
            }

            // Add GnssSats to NMEA message
            msg.gnss_sats.push_back(sats_msg);
        }

        // Clear map
        nmea_message_.gnss_signals.clear();

        // True heading
        msg.heading = nmea_message_.heading;

        // Speed over ground [m/s]
        msg.speed = nmea_message_.speed;

        // Course over ground [deg]
        msg.course = nmea_message_.course;

        // Populate differential data information
        msg.diff_age = nmea_message_.diff_age;
        msg.diff_sta = nmea_message_.diff_sta;

        // Publish message
        nmea_pub_.publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::WsCallbackRos(const fixposition_driver_ros1::SpeedConstPtr& msg) {
    std::unordered_map<std::string, std::vector<std::pair<bool, int>>> measurements;
    for (const auto& sensor : msg->sensors) {
        measurements[sensor.location].push_back({sensor.vx_valid, sensor.vx});
        measurements[sensor.location].push_back({sensor.vy_valid, sensor.vy});
        measurements[sensor.location].push_back({sensor.vz_valid, sensor.vz});
    }
    WsCallback(measurements);
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::RtcmCallbackRos(const rtcm_msgs::MessageConstPtr& msg) {
    RtcmCallback(msg->message.data(), msg->message.size());
}
#endif

}  // namespace fixposition
/* ****************************************************************************************************************** */

using namespace fixposition;

int main(int argc, char** argv) {
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
    WARNING("***** Running debug build *****");
#endif

    bool ok = true;

    ros::init(argc, argv, "fixposition_driver");
    ros::NodeHandle node_handle("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // Redirect Fixposition SDK logging to ROS console (the logger for these is "ros.fpsdk_ros1")
    fpsdk::ros1::utils::RedirectLoggingToRosConsole();

    // Load parameters
    ROS_INFO("Loading parameters...");
    SensorParams sensor_params;
    if (!LoadParamsFromRos1("~", sensor_params)) {
        ROS_ERROR("Failed loading sensor params");
        ok = false;
    }
    NodeParams node_params;
    if (!LoadParamsFromRos1("~", node_params)) {
        ROS_ERROR("Failed loading node params");
        ok = false;
    }

    // Handle CTRL-C / SIGINT ourselves
    fpsdk::common::app::SigIntHelper sigint;

    // Start node
    std::unique_ptr<FixpositionDriverNode> node;
    if (ok) {
        try {
            node = std::make_unique<FixpositionDriverNode>(sensor_params, node_params, node_handle);
        } catch (const std::exception& ex) {
            ROS_ERROR("Failed creating node: %s", ex.what());
            ok = false;
        }
    }
    if (ok) {
        ROS_INFO("Starting node...");
        if (node->StartNode()) {
            ROS_INFO("main() spinning...");
            // Use multiple spinner threads. Callback execute in one of them.
            ros::AsyncSpinner spinner(2);
            spinner.start();
            sigint.WaitAbort();
            spinner.stop();
            ROS_INFO("main() stopping");
        } else {
            ROS_ERROR("Failed starting node");
            ok = false;
        }
        node.reset();
    }

    // Are we happy?
    if (ok) {
        ROS_INFO("Done");
    } else {
        ROS_FATAL("Ouch!");
    }

    // Shutdown ROS
    ros::shutdown();

    exit(ok ? EXIT_SUCCESS : EXIT_FAILURE);
}

/* ****************************************************************************************************************** */
