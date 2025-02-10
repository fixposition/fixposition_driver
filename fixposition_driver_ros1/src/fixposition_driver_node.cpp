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
 * @brief Fixposition driver node for ROS1
 */

/* LIBC/STL */
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

/* EXTERNAL */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/nmea.hpp>
#include <fpsdk_common/parser/novb.hpp>
#include <fpsdk_common/trafo.hpp>
#include <fpsdk_common/types.hpp>
#include <fpsdk_ros1/ext/eigen_conversions.hpp>
#include <fpsdk_ros1/utils.hpp>

/* PACKAGE */
#include "fixposition_driver_ros1/fixposition_driver_node.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::common;
using namespace fpsdk::common::parser;

FixpositionDriverNode::FixpositionDriverNode(const DriverParams& params, ros::NodeHandle& nh) /* clang-format off */ :
    nh_                { nh },
    params_            { params },
    driver_            { params_ },
    nmea_epoch_data_   { params_.nmea_epoch_ }  // clang-format on
{}

FixpositionDriverNode::~FixpositionDriverNode() { StopNode(); }

// ---------------------------------------------------------------------------------------------------------------------

// Helper for advertising output topics
#define _PUB(_pub_, _type_, _topic_, ...)                          \
    if (_pub_.getTopic().empty()) {                                \
        ROS_INFO("Advertise %s (" #_type_ ")", (_topic_).c_str()); \
        _pub_ = nh_.advertise<_type_>(_topic_, __VA_ARGS__);       \
    }

// Helper for subscribing to input topics
#define _SUB(_sub_, _type_, _topic_, ...)                          \
    do {                                                           \
        ROS_INFO("Subscribe %s (" #_type_ ")", (_topic_).c_str()); \
        _sub_ = nh_.subscribe<_type_>(_topic_, __VA_ARGS__);       \
    } while (0)

bool FixpositionDriverNode::StartNode() {
    ROS_INFO("Starting...");

    // Add observers and advertise output topics, depending on configuration
    const std::string output_ns = (params_.output_ns_.empty() ? nh_.getNamespace() : params_.output_ns_);

    // FP_A-ODOMETRY
    if (params_.MessageEnabled(fpa::FpaOdometryPayload::MSG_NAME)) {
        _PUB(fpa_odometry_pub_, fixposition_driver_msgs::FpaOdometry, output_ns + "/fpa/odometry", 5);
        _PUB(odometry_ecef_pub_, nav_msgs::Odometry, output_ns + "/odometry_ecef", 5);
        _PUB(odometry_llh_pub_, sensor_msgs::NavSatFix, output_ns + "/odometry_llh", 5);
        _PUB(poiimu_pub_, sensor_msgs::Imu, output_ns + "/poiimu", 5);
        driver_.AddFpaObserver(fpa::FpaOdometryPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto odometry_payload = dynamic_cast<const fpa::FpaOdometryPayload&>(payload);
            PublishFpaOdometry(odometry_payload, fpa_odometry_pub_);
            PublishFpaOdometryDataImu(odometry_payload, poiimu_pub_);
            PublishFpaOdometryDataNavSatFix(odometry_payload, odometry_llh_pub_);
            OdometryData odometry_data;
            odometry_data.SetFromFpaOdomPayload(odometry_payload);
            PublishOdometryData(odometry_data, odometry_ecef_pub_);
            ProcessOdometryData(odometry_data);
            fusion_epoch_data_.CollectFpaOdometry(odometry_payload);
        });
    }

    // FP_A-ODOMSH
    if (params_.MessageEnabled(fpa::FpaOdomshPayload::MSG_NAME)) {
        _PUB(fpa_odomsh_pub_, fixposition_driver_msgs::FpaOdomsh, output_ns + "/fpa/odomsh", 5);
        _PUB(odometry_smooth_pub_, nav_msgs::Odometry, output_ns + "/odometry_smooth", 5);
        driver_.AddFpaObserver(fpa::FpaOdomshPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto odomsh_payload = dynamic_cast<const fpa::FpaOdomshPayload&>(payload);
            PublishFpaOdomsh(odomsh_payload, fpa_odomsh_pub_);
            OdometryData odometry_data;
            odometry_data.SetFromFpaOdomPayload(odomsh_payload);
            PublishOdometryData(odometry_data, odometry_smooth_pub_);
            ProcessOdometryData(odometry_data);
            fusion_epoch_data_.CollectFpaOdomsh(odomsh_payload);
        });
    }

    // FP_A-ODOMENU
    if (params_.MessageEnabled(fpa::FpaOdomenuPayload::MSG_NAME)) {
        _PUB(fpa_odomenu_pub_, fixposition_driver_msgs::FpaOdomenu, output_ns + "/fpa/odomenu", 5);
        _PUB(odometry_enu_pub_, nav_msgs::Odometry, output_ns + "/odometry_enu", 5);
        _PUB(eul_pub_, geometry_msgs::Vector3Stamped, output_ns + "/ypr", 5);
        driver_.AddFpaObserver(fpa::FpaOdomenuPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto odomenu_payload = dynamic_cast<const fpa::FpaOdomenuPayload&>(payload);
            PublishFpaOdomenu(odomenu_payload, fpa_odomenu_pub_);
            PublishFpaOdomenuVector3Stamped(odomenu_payload, eul_pub_);
            OdometryData odometry_data;
            odometry_data.SetFromFpaOdomPayload(odomenu_payload);
            PublishOdometryData(odometry_data, odometry_enu_pub_);
            ProcessOdometryData(odometry_data);
            fusion_epoch_data_.CollectFpaOdomenu(odomenu_payload);
        });
    }

    // FP_A-ODOMSTATUS
    if (params_.MessageEnabled(fpa::FpaOdomstatusPayload::MSG_NAME)) {
        _PUB(fpa_odomstatus_pub_, fixposition_driver_msgs::FpaOdomstatus, output_ns + "/fpa/odomstatus", 5);
        driver_.AddFpaObserver(fpa::FpaOdomstatusPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto odomstatus_payload = dynamic_cast<const fpa::FpaOdomstatusPayload&>(payload);
            PublishFpaOdomstatus(odomstatus_payload, fpa_odomstatus_pub_);
            fusion_epoch_data_.CollectFpaOdomstatus(odomstatus_payload);
        });
    }

    // FP_A-EOE
    if (params_.MessageEnabled(fpa::FpaEoePayload::MSG_NAME)) {
        _PUB(fpa_eoe_pub_, fixposition_driver_msgs::FpaEoe, output_ns + "/fpa/eoe", 5);
        driver_.AddFpaObserver(fpa::FpaEoePayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto eoe_payload = dynamic_cast<const fpa::FpaEoePayload&>(payload);
            PublishFpaEoe(eoe_payload, fpa_eoe_pub_);
            if (eoe_payload.epoch == fpa::FpaEpoch::FUSION) {
                // Fusion epoch
                PublishFusionEpochData(fusion_epoch_data_.CompleteAndReset(eoe_payload), fusion_epoch_pub_);
                // Generate Nav2 TF tree
                if (params_.nav2_mode_) {
                    PublishNav2Tf();
                }
            }
            
            // NMEA epoch (which can be FUSION, too)
            if (params_.nmea_epoch_ == eoe_payload.epoch) {
                PublishNmeaEpochData(nmea_epoch_data_.CompleteAndReset(), nmea_epoch_pub_);
            }
        });
    }

    // FP_A-TF
    if (params_.MessageEnabled(fpa::FpaTfPayload::MSG_NAME)) {
        _PUB(eul_imu_pub_, geometry_msgs::Vector3Stamped, output_ns + "/imu_ypr", 5);
        driver_.AddFpaObserver(fpa::FpaTfPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            TfData tf;
            tf.SetFromFpaTfPayload(dynamic_cast<const fpa::FpaTfPayload&>(payload));
            ProcessTfData(tf);
        });
    }

    // FP_A-LLH
    if (params_.MessageEnabled(fpa::FpaLlhPayload::MSG_NAME)) {
        _PUB(fpa_llh_pub_, fixposition_driver_msgs::FpaLlh, output_ns + "/fpa/llh", 5);
        driver_.AddFpaObserver(fpa::FpaLlhPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaLlh(dynamic_cast<const fpa::FpaLlhPayload&>(payload), fpa_llh_pub_);
        });
    }

    // FP_A-GNSSANT
    if (params_.MessageEnabled(fpa::FpaGnssantPayload::MSG_NAME)) {
        _PUB(fpa_gnssant_pub_, fixposition_driver_msgs::FpaGnssant, output_ns + "/fpa/gnssant", 5);
        driver_.AddFpaObserver(fpa::FpaGnssantPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaGnssant(dynamic_cast<const fpa::FpaGnssantPayload&>(payload), fpa_gnssant_pub_);
        });
    }

    // FP_A-GNSSCORR
    if (params_.MessageEnabled(fpa::FpaGnsscorrPayload::MSG_NAME)) {
        _PUB(fpa_gnsscorr_pub_, fixposition_driver_msgs::FpaGnsscorr, output_ns + "/fpa/gnsscorr", 5);
        driver_.AddFpaObserver(fpa::FpaGnsscorrPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaGnsscorr(dynamic_cast<const fpa::FpaGnsscorrPayload&>(payload), fpa_gnsscorr_pub_);
        });
    }

    // FP_A-IMUBIAS
    if (params_.MessageEnabled(fpa::FpaImubiasPayload::MSG_NAME)) {
        _PUB(fpa_imubias_pub_, fixposition_driver_msgs::FpaImubias, output_ns + "/fpa/imubias", 5);
        driver_.AddFpaObserver(fpa::FpaImubiasPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto imubias_payload = dynamic_cast<const fpa::FpaImubiasPayload&>(payload);
            PublishFpaImubias(imubias_payload, fpa_imubias_pub_);
            fusion_epoch_data_.CollectFpaImubias(imubias_payload);
        });
    }

    // FP_A-RAWIMU
    if (params_.MessageEnabled(fpa::FpaRawimuPayload::MSG_NAME)) {
        _PUB(rawimu_pub_, sensor_msgs::Imu, output_ns + "/fpa/rawimu", 5);
        driver_.AddFpaObserver(fpa::FpaRawimuPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaRawimu(dynamic_cast<const fpa::FpaRawimuPayload&>(payload), rawimu_pub_);
        });
    }

    // FP_A-CORRIMU
    if (params_.MessageEnabled(fpa::FpaCorrimuPayload::MSG_NAME)) {
        _PUB(corrimu_pub_, sensor_msgs::Imu, output_ns + "/fpa/corrimu", 5);
        driver_.AddFpaObserver(fpa::FpaCorrimuPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaCorrimu(dynamic_cast<const fpa::FpaCorrimuPayload&>(payload), corrimu_pub_);
        });
    }

    // FP_A-TEXT
    if (params_.MessageEnabled(fpa::FpaTextPayload::MSG_NAME)) {
        _PUB(fpa_text_pub_, fixposition_driver_msgs::FpaText, output_ns + "/fpa/text", 5);
        driver_.AddFpaObserver(fpa::FpaTextPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaText(dynamic_cast<const fpa::FpaTextPayload&>(payload), fpa_text_pub_);
        });
    }

    // FP_A-TP
    if (params_.MessageEnabled(fpa::FpaTpPayload::MSG_NAME)) {
        _PUB(fpa_tp_pub_, fixposition_driver_msgs::FpaTp, output_ns + "/fpa/tp", 5);
        driver_.AddFpaObserver(fpa::FpaTpPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaTp(dynamic_cast<const fpa::FpaTpPayload&>(payload), fpa_tp_pub_);
        });
    }

    // NOV_B-BESTGNSSPOS
    if (params_.MessageEnabled(novb::NOV_B_BESTGNSSPOS_STRID)) {
        _PUB(navsatfix_gnss1_pub_, sensor_msgs::NavSatFix, output_ns + "/gnss1", 5);
        _PUB(navsatfix_gnss2_pub_, sensor_msgs::NavSatFix, output_ns + "/gnss2", 5);
        driver_.AddNovbObserver(  //
            novb::NOV_B_BESTGNSSPOS_STRID, [this](const novb::NovbHeader* header, const uint8_t* payload) {
                if (!PublishNovbBestgnsspos(header, (novb::NovbBestgnsspos*)payload, navsatfix_gnss1_pub_,
                                            navsatfix_gnss2_pub_)) {
                    ROS_WARN_THROTTLE(1.0, "Bad NOV_B-BESTGNSSPOS");
                }
            });
    }

    // NOV_B-INSPVAX
    if (params_.MessageEnabled(novb::NOV_B_INSPVAX_STRID)) {
        _PUB(novb_inspvax_pub_, fixposition_driver_msgs::NovbInspvax, output_ns + "/novb/inspvax", 5);
        driver_.AddNovbObserver(  //
            novb::NOV_B_INSPVAX_STRID, [this](const novb::NovbHeader* header, const uint8_t* payload) {
                if (!PublishNovbInspvax(header, (novb::NovbInspvax*)payload, novb_inspvax_pub_)) {
                    ROS_WARN_THROTTLE(1.0, "Bad NOV_B-INSPVAX");
                }
                fusion_epoch_data_.CollectNovbInspvax(header, (novb::NovbInspvax*)payload);
            });
    }

    // NMEA-GP-GGA
    if (params_.MessageEnabled(nmea::NmeaGgaPayload::FORMATTER)) {
        _PUB(nmea_gga_pub_, fixposition_driver_msgs::NmeaGga, output_ns + "/nmea/gga", 5);
        driver_.AddNmeaObserver(nmea::NmeaGgaPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gga_payload = dynamic_cast<const nmea::NmeaGgaPayload&>(payload);
            PublishNmeaGga(gga_payload, nmea_gga_pub_);
            nmea_epoch_data_.gga_ = gga_payload;
        });
    }

    // NMEA-GP-GLL
    if (params_.MessageEnabled(nmea::NmeaGllPayload::FORMATTER)) {
        _PUB(nmea_gll_pub_, fixposition_driver_msgs::NmeaGll, output_ns + "/nmea/gll", 5);
        driver_.AddNmeaObserver(nmea::NmeaGllPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gll_payload = dynamic_cast<const nmea::NmeaGllPayload&>(payload);
            PublishNmeaGll(gll_payload, nmea_gll_pub_);
            nmea_epoch_data_.gll_ = gll_payload;
        });
    }

    // NMEA-GN-GSA
    if (params_.MessageEnabled(nmea::NmeaGsaPayload::FORMATTER)) {
        _PUB(nmea_gsa_pub_, fixposition_driver_msgs::NmeaGsa, output_ns + "/nmea/gsa", 5);
        driver_.AddNmeaObserver(nmea::NmeaGsaPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gsa_payload_ = dynamic_cast<const nmea::NmeaGsaPayload&>(payload);
            PublishNmeaGsa(gsa_payload_, nmea_gsa_pub_);
            nmea_epoch_data_.gsa_ = gsa_payload_;
            nmea_epoch_data_.gsa_gsv_.AddGsa(gsa_payload_);
        });
    }

    // NMEA-GP-GST
    if (params_.MessageEnabled(nmea::NmeaGstPayload::FORMATTER)) {
        _PUB(nmea_gst_pub_, fixposition_driver_msgs::NmeaGst, output_ns + "/nmea/gst", 5);
        driver_.AddNmeaObserver(nmea::NmeaGstPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gst_payload = dynamic_cast<const nmea::NmeaGstPayload&>(payload);
            PublishNmeaGst(gst_payload, nmea_gst_pub_);
            nmea_epoch_data_.gst_ = gst_payload;
        });
    }

    // NMEA-GX-GSV
    if (params_.MessageEnabled(nmea::NmeaGsvPayload::FORMATTER)) {
        _PUB(nmea_gsv_pub_, fixposition_driver_msgs::NmeaGsv, output_ns + "/nmea/gsv", 5);
        driver_.AddNmeaObserver(nmea::NmeaGsvPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gsv_payload_ = dynamic_cast<const nmea::NmeaGsvPayload&>(payload);
            PublishNmeaGsv(gsv_payload_, nmea_gsv_pub_);
            nmea_epoch_data_.gsa_gsv_.AddGsv(gsv_payload_);
        });
    }

    // NMEA-GP-HDT
    if (params_.MessageEnabled(nmea::NmeaHdtPayload::FORMATTER)) {
        _PUB(nmea_hdt_pub_, fixposition_driver_msgs::NmeaHdt, output_ns + "/nmea/hdt", 5);
        driver_.AddNmeaObserver(nmea::NmeaHdtPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto hdt_payload = dynamic_cast<const nmea::NmeaHdtPayload&>(payload);
            PublishNmeaHdt(hdt_payload, nmea_hdt_pub_);
            nmea_epoch_data_.hdt_ = hdt_payload;
        });
    }

    // NMEA-GP-RMC
    if (params_.MessageEnabled(nmea::NmeaRmcPayload::FORMATTER)) {
        _PUB(nmea_rmc_pub_, fixposition_driver_msgs::NmeaRmc, output_ns + "/nmea/rmc", 5);
        driver_.AddNmeaObserver(nmea::NmeaRmcPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto rmc_payload = dynamic_cast<const nmea::NmeaRmcPayload&>(payload);
            PublishNmeaRmc(rmc_payload, nmea_rmc_pub_);
            nmea_epoch_data_.rmc_ = rmc_payload;
        });
    }

    // NMEA-GP-VTG
    if (params_.MessageEnabled(nmea::NmeaVtgPayload::FORMATTER)) {
        _PUB(nmea_vtg_pub_, fixposition_driver_msgs::NmeaVtg, output_ns + "/nmea/vtg", 5);
        driver_.AddNmeaObserver(nmea::NmeaVtgPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto vtg_payload = dynamic_cast<const nmea::NmeaVtgPayload&>(payload);
            PublishNmeaVtg(vtg_payload, nmea_vtg_pub_);
            nmea_epoch_data_.vtg_ = vtg_payload;
        });
    }

    // NMEA-GP-ZDA
    if (params_.MessageEnabled(nmea::NmeaZdaPayload::FORMATTER)) {
        _PUB(nmea_zda_pub_, fixposition_driver_msgs::NmeaZda, output_ns + "/nmea/zda", 5);
        driver_.AddNmeaObserver(nmea::NmeaZdaPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto zda_payload = dynamic_cast<const nmea::NmeaZdaPayload&>(payload);
            PublishNmeaZda(zda_payload, nmea_zda_pub_);
            nmea_epoch_data_.zda_ = zda_payload;
        });
    }

    // Raw messages
    if (params_.raw_output_) {
        _PUB(raw_pub_, fixposition_driver_msgs::ParserMsg, output_ns + "/raw", 5);
        driver_.AddRawObserver([this](const parser::ParserMsg& msg) { PublishParserMsg(msg, raw_pub_); });
    }

    // Fusion epoch
    if (params_.fusion_epoch_) {
        _PUB(fusion_epoch_pub_, fixposition_driver_msgs::FusionEpoch, output_ns + "/fusion", 5);
        // Publish is triggered by FP_A-EOE above
    }

    // NMEA epoch
    if (params_.nmea_epoch_ != fpa::FpaEpoch::UNSPECIFIED) {
        _PUB(nmea_epoch_pub_, fixposition_driver_msgs::NmeaEpoch, output_ns + "/nmea", 5);
        // Publish is triggered by FP_A-EOE above
    }

    // Jump warning message
    if (params_.cov_warning_) {
        _PUB(jump_pub_, fixposition_driver_msgs::CovWarn, output_ns + "/extras/jump", 5);
    }

    // Subscribe to correction data input
    if (!params_.corr_topic_.empty()) {
        _SUB(corr_sub_, rtcm_msgs::Message, params_.corr_topic_, 100, [this](const rtcm_msgs::MessageConstPtr& msg) {
            driver_.SendCorrectionData(msg->message.data(), msg->message.size());
        });
    }

    // Subscribe to wheelspeed input
    if (!params_.speed_topic_.empty()) {
        _SUB(ws_sub_, fixposition_driver_msgs::Speed, params_.speed_topic_, 10,
             [this](const fixposition_driver_msgs::SpeedConstPtr& msg) {
                 driver_.SendWheelspeedData(SpeedMsgToWheelspeedData(*msg));
             });
    }

    // Subscribe to wheelspeed input
    if (params_.converter_enabled_) {
        if (!params_.converter_input_topic_.empty()) {
            switch (params_.converter_topic_type_) {
                case DriverParams::VelTopicType::TWIST:
                    _SUB(ws_conv_sub_, geometry_msgs::Twist, params_.converter_input_topic_, 10,
                         [this](const geometry_msgs::TwistConstPtr& msg) {
                             driver_.SendWheelspeedData(SpeedConverterCallback(*msg, params_));
                         });
                    break;
                case DriverParams::VelTopicType::TWISTWITHCOV:
                    _SUB(ws_conv_sub_, geometry_msgs::TwistWithCovariance, params_.converter_input_topic_, 10,
                         [this](const geometry_msgs::TwistWithCovarianceConstPtr& msg) {
                             driver_.SendWheelspeedData(SpeedConverterCallback(*msg, params_));
                         });
                    break;
                case DriverParams::VelTopicType::ODOMETRY:
                    _SUB(ws_conv_sub_, nav_msgs::Odometry, params_.converter_input_topic_, 10,
                         [this](const nav_msgs::OdometryConstPtr& msg) {
                             driver_.SendWheelspeedData(SpeedConverterCallback(*msg, params_));
                         });
                    break;
                default:
                    ROS_WARN_THROTTLE(1.0, "The selected wheelspeed input type is not supported!");
                    break;
            }
        }
    }

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
    // - FP_A messages
    fpa_eoe_pub_.shutdown();
    fpa_gnssant_pub_.shutdown();
    fpa_gnsscorr_pub_.shutdown();
    fpa_imubias_pub_.shutdown();
    fpa_llh_pub_.shutdown();
    fpa_odomenu_pub_.shutdown();
    fpa_odometry_pub_.shutdown();
    fpa_odomsh_pub_.shutdown();
    fpa_odomstatus_pub_.shutdown();
    fpa_text_pub_.shutdown();
    fpa_tp_pub_.shutdown();
    // - NMEA messages
    nmea_gga_pub_.shutdown();
    nmea_gll_pub_.shutdown();
    nmea_gsa_pub_.shutdown();
    nmea_gst_pub_.shutdown();
    nmea_gsv_pub_.shutdown();
    nmea_hdt_pub_.shutdown();
    nmea_rmc_pub_.shutdown();
    nmea_vtg_pub_.shutdown();
    nmea_zda_pub_.shutdown();
    // - NOV_B messages
    novb_inspvax_pub_.shutdown();
    // - Odometry
    odometry_ecef_pub_.shutdown();
    odometry_enu_pub_.shutdown();
    odometry_llh_pub_.shutdown();
    odometry_smooth_pub_.shutdown();
    // - Orientation
    eul_pub_.shutdown();
    eul_imu_pub_.shutdown();
    // - IMU
    corrimu_pub_.shutdown();
    poiimu_pub_.shutdown();
    rawimu_pub_.shutdown();
    // - GNSS
    navsatfix_gnss1_pub_.shutdown();
    navsatfix_gnss2_pub_.shutdown();
    fusion_epoch_pub_.shutdown();
    nmea_epoch_pub_.shutdown();
    // - Other
    jump_pub_.shutdown();
    raw_pub_.shutdown();

    // Stop input message subscribers
    ws_sub_.shutdown();
    corr_sub_.shutdown();
    ws_conv_sub_.shutdown();
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::ProcessTfData(const TfData& tf_data) {
    geometry_msgs::TransformStamped tf;
    TfDataToTransformStamped(tf_data, tf);

    // TODO: use constants from helper.hpp?

    if ((tf.child_frame_id == "FP_IMUH") && (tf.header.frame_id == "FP_POI")) {
        tf_br_.sendTransform(tf);

        // Publish pitch roll based on IMU only
        Eigen::Vector3d imu_ypr_eigen = trafo::QuatToEul(tf_data.rotation);
        imu_ypr_eigen.x() = 0.0;  // the yaw value is not observable using IMU alone
        geometry_msgs::Vector3Stamped imu_ypr;
        imu_ypr.header.stamp = tf.header.stamp;
        imu_ypr.header.frame_id = "FP_POI";
        tf::vectorEigenToMsg(imu_ypr_eigen, imu_ypr.vector);
        eul_imu_pub_.publish(imu_ypr);

    }
    // FP_POI -> FP_POISH
    else if ((tf.child_frame_id == "FP_POISH") && (tf.header.frame_id == "FP_POI")) {
        tf_br_.sendTransform(tf);
        // Store TF if Nav2 mode is enabled
        if (params_.nav2_mode_) {
            std::unique_lock<std::mutex> lock(tfs_.mutex_);
            tfs_.poi_poish_ = std::make_unique<geometry_msgs::TransformStamped>(tf);
        }
    }
    // FP_ECEF -> FP_ENU0
    else if ((tf.child_frame_id == "FP_ENU0") && (tf.header.frame_id == "FP_ECEF")) {
        static_br_.sendTransform(tf);
        // Store TF if Nav2 mode is enabled
        if (params_.nav2_mode_) {
            std::unique_lock<std::mutex> lock(tfs_.mutex_);
            tfs_.ecef_enu0_ = std::make_unique<geometry_msgs::TransformStamped>(tf);
        }
    }
    // Something else
    else {
        static_br_.sendTransform(tf);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::ProcessOdometryData(const OdometryData& odometry_data) {
    // Send a warning if the system experiences delays
    // This message computes the difference between the message time and the local system time.
    // Thus, if the local time is off, the message might be triggered or not triggered when it should.
    if (params_.delay_warning_ > 0.0) {
        const double delay = (ros::Time::now() - fpsdk::ros1::utils::ConvTime(odometry_data.stamp)).toSec();
        if (delay > params_.delay_warning_) {
            ROS_WARN_THROTTLE(1.0, "The system is experiencing significant delays! (estimated delay: %.3f seconds)",
                              delay);
        }
    }

    switch (odometry_data.type) {
        case OdometryData::Type::ODOMETRY:

            if (odometry_data.valid) {
                geometry_msgs::TransformStamped tf;
                OdometryDataToTransformStamped(odometry_data, tf);
                tf_br_.sendTransform(tf);
            }

            // Output jump warning
            if (params_.cov_warning_ && odometry_data.valid && jump_detector_.Check(odometry_data)) {
                ROS_WARN(jump_detector_.warning_.c_str());
                PublishJumpWarning(jump_detector_, jump_pub_);
            }

            break;

        case OdometryData::Type::ODOMENU:
            // Store FP_ENU0 -> FP_POI TF if Nav2 mode is selected
            if (params_.nav2_mode_ && odometry_data.valid) {
                std::unique_lock<std::mutex> lock(tfs_.mutex_);
                tfs_.enu0_poi_ = std::make_unique<geometry_msgs::TransformStamped>();
                OdometryDataToTransformStamped(odometry_data, *tfs_.enu0_poi_);
            }
            break;

        case OdometryData::Type::ODOMSH:
            // Store FP_ECEF -> FP_POISH TF if Nav2 mode is selected
            if (params_.nav2_mode_ && odometry_data.valid) {
                std::unique_lock<std::mutex> lock(tfs_.mutex_);
                tfs_.ecef_poish_ = std::make_unique<geometry_msgs::TransformStamped>();
                OdometryDataToTransformStamped(odometry_data, *tfs_.ecef_poish_);
            }
            break;

        case OdometryData::Type::UNSPECIFIED:
            break;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::PublishNav2Tf() {
    std::unique_lock<std::mutex> lock(tfs_.mutex_);
    // We'll need all before we can start publishing the Nav2 TFs
    if (!tfs_.ecef_enu0_ || !tfs_.poi_poish_ || !tfs_.ecef_poish_ || !tfs_.enu0_poi_) {
        return;
    }

    // Publish FP_ECEF -> map
    tfs_.ecef_enu0_->child_frame_id = "map";
    static_br_.sendTransform(*tfs_.ecef_enu0_);

    // Compute FP_ENU0 -> FP_POISH
    // Extract translation and rotation from ECEFENU0
    geometry_msgs::Vector3 trans_ecef_enu0 = tfs_.ecef_enu0_->transform.translation;
    geometry_msgs::Quaternion rot_ecef_enu0 = tfs_.ecef_enu0_->transform.rotation;
    Eigen::Vector3d t_ecef_enu0_;
    t_ecef_enu0_ << trans_ecef_enu0.x, trans_ecef_enu0.y, trans_ecef_enu0.z;
    Eigen::Quaterniond q_ecef_enu0_(rot_ecef_enu0.w, rot_ecef_enu0.x, rot_ecef_enu0.y, rot_ecef_enu0.z);

    // Extract translation and rotation from ECEFPOISH
    geometry_msgs::Vector3 trans_ecef_poish = tfs_.ecef_poish_->transform.translation;
    geometry_msgs::Quaternion rot_ecef_poish = tfs_.ecef_poish_->transform.rotation;
    Eigen::Vector3d t_ecef_poish;
    t_ecef_poish << trans_ecef_poish.x, trans_ecef_poish.y, trans_ecef_poish.z;
    Eigen::Quaterniond q_ecef_poish(rot_ecef_poish.w, rot_ecef_poish.x, rot_ecef_poish.y, rot_ecef_poish.z);

    // Compute the ENU transformation
    const Eigen::Vector3d t_enu0_poish = trafo::TfEnuEcef(t_ecef_poish, trafo::TfWgs84LlhEcef(t_ecef_enu0_));
    const Eigen::Quaterniond q_enu0_poish = q_ecef_enu0_.inverse() * q_ecef_poish;

    // Create tf2::Transform tf_ENU0POISH
    tf2::Transform tf_ENU0POISH;
    tf_ENU0POISH.setOrigin(tf2::Vector3(t_enu0_poish.x(), t_enu0_poish.y(), t_enu0_poish.z()));
    tf2::Quaternion tf_q_enu0_poish(q_enu0_poish.x(), q_enu0_poish.y(), q_enu0_poish.z(), q_enu0_poish.w());
    tf_ENU0POISH.setRotation(tf_q_enu0_poish);

    // Publish map -> odom
    // Multiply the transforms
    tf2::Transform tf_ENU0POI;
    tf2::fromMsg(tfs_.enu0_poi_->transform, tf_ENU0POI);
    tf2::Transform tf_combined = tf_ENU0POI * tf_ENU0POISH.inverse();

    // Create a new TransformStamped message
    geometry_msgs::TransformStamped tfs_odom;
    tfs_odom.header.stamp = ros::Time::now();
    tfs_odom.header.frame_id = "map";
    tfs_odom.child_frame_id = "odom";
    tfs_odom.transform = tf2::toMsg(tf_combined);
    tf_br_.sendTransform(tfs_odom);

    // Publish odom -> base_link
    geometry_msgs::TransformStamped tf_odom_base;
    tf_odom_base.header.stamp = ros::Time::now();
    tf_odom_base.header.frame_id = "odom";
    tf_odom_base.child_frame_id = "base_link";
    tf_odom_base.transform = tf2::toMsg(tf_ENU0POISH);

    // Send the transform
    tf_br_.sendTransform(tf_odom_base);
}

/* ****************************************************************************************************************** */
}  // namespace fixposition

using namespace fixposition;

int main(int argc, char** argv) {
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
    WARNING("***** Running debug build *****");
#endif

    bool ok = true;

    // Initialise ROS, create node handle
    ros::init(argc, argv, "fixposition_driver");
    ros::NodeHandle node_handle("~");

    // Redirect Fixposition SDK logging to ROS console
    fpsdk::ros1::utils::RedirectLoggingToRosConsole();

    // Say hello
    HelloWorld();

    // Load parameters
    ROS_INFO("Loading parameters...");
    DriverParams driver_params;
    if (!LoadParamsFromRos1("~", driver_params)) {
        ROS_ERROR("Failed loading sensor params");
        ok = false;
    }

    // Handle CTRL-C / SIGINT ourselves
    fpsdk::common::app::SigIntHelper sigint;

    // Start node
    std::unique_ptr<FixpositionDriverNode> node;
    if (ok) {
        try {
            node = std::make_unique<FixpositionDriverNode>(driver_params, node_handle);
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
            ros::AsyncSpinner spinner(4);
            spinner.start();
            sigint.WaitAbort();
            spinner.stop();
            ROS_INFO("main() stopping");
        } else {
            ROS_ERROR("Failed starting node");
            ok = false;
        }
        node->StopNode();
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
