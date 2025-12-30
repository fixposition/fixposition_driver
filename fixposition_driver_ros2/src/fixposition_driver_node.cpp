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
 * @brief Fixposition driver node for ROS2
 */

/* LIBC/STL */
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

/* EXTERNAL */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/nmea.hpp>
#include <fpsdk_common/parser/novb.hpp>
#include <fpsdk_common/trafo.hpp>
#include <fpsdk_common/types.hpp>
#include <fpsdk_ros2/utils.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/create_timer.hpp>

/* PACKAGE */
#include "fixposition_driver_ros2/fixposition_driver_node.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::common;
using namespace fpsdk::common::parser;

namespace {
void AddFpaInt(diagnostic_updater::DiagnosticStatusWrapper& status, const std::string& key,
               const fpa::FpaInt& value) {
    status.add(key + "_valid", value.valid);
    status.add(key, value.valid ? value.value : -1);
}

void AddFpaFloat(diagnostic_updater::DiagnosticStatusWrapper& status, const std::string& key,
                 const fpa::FpaFloat& value) {
    status.add(key + "_valid", value.valid);
    status.add(key, value.valid ? value.value : std::numeric_limits<double>::quiet_NaN());
}
}  // namespace

FixpositionDriverNode::FixpositionDriverNode(NodeInterfaces node_interfaces, const DriverParams& params,
                                             const DiagnosticsParams& diagnostics_params) /* clang-format off */ :
    node_interfaces_   { std::move(node_interfaces) },
    params_            { params },
    logger_            { node_interfaces_.logging_->get_logger() },
    driver_            { params },
    qos_settings_      { rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default) },
    diagnostics_params_ { diagnostics_params },
    clock_             { node_interfaces_.clock_->get_clock() },
    nmea_epoch_data_   { params_.nmea_epoch_ }  // clang-format on
{
    // Override default QoS settings
    // - Short-queue sensor-type QoS
    if (params_.qos_type_ == "sensor_short") {
        qos_settings_ = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data);
    }
    // - Long-queue sensor-type QoS
    else if (params_.qos_type_ == "sensor_long") {
        qos_settings_ = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    }
    // - Short-queue default-type QoS
    else if (params_.qos_type_ == "default_short") {
        qos_settings_ = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_default);
    }
    // - Long-queue default-type QoS
    else if (params_.qos_type_ == "default_long") {
        qos_settings_ = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default);
    }
}

FixpositionDriverNode::~FixpositionDriverNode() {}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::ResetDiagnosticsState() {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    have_rx_time_ = false;
    have_odomstatus_ = false;
    have_gnssant_ = false;
    have_gnsscorr_ = false;
    have_text_ = false;
    last_text_string_.clear();
}

void FixpositionDriverNode::RecordRxTime() {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    last_rx_time_ = clock_->now();
    have_rx_time_ = true;
}

void FixpositionDriverNode::StoreOdomstatus(const fpa::FpaOdomstatusPayload& payload) {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    last_odomstatus_ = payload;
    have_odomstatus_ = true;
}

void FixpositionDriverNode::StoreGnssant(const fpa::FpaGnssantPayload& payload) {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    last_gnssant_ = payload;
    have_gnssant_ = true;
}

void FixpositionDriverNode::StoreGnsscorr(const fpa::FpaGnsscorrPayload& payload) {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    last_gnsscorr_ = payload;
    have_gnsscorr_ = true;
}

void FixpositionDriverNode::StoreText(const fpa::FpaTextPayload& payload) {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    last_text_ = payload;
    last_text_string_ = payload.text;
    have_text_ = true;
}

void FixpositionDriverNode::DiagnosticsDriver(diagnostic_updater::DiagnosticStatusWrapper& status) {
    rclcpp::Time now = clock_->now();
    bool running = false;
    bool have_rx = false;
    rclcpp::Time last_rx;

    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        running = driver_running_;
        have_rx = have_rx_time_;
        last_rx = last_rx_time_;
    }

    if (!running) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Driver stopped");
    } else if (!have_rx) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data received yet");
    } else {
        double age_ms = (now - last_rx).seconds() * 1e3;
        if (age_ms > diagnostics_params_.timeout_ms_) {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Data timeout");
        } else {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Driver running");
        }
        status.add("last_msg_age_ms", age_ms);
    }

    status.add("driver_running", running ? "true" : "false");
    status.add("timeout_ms", diagnostics_params_.timeout_ms_);
}

void FixpositionDriverNode::DiagnosticsFusion(diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (!params_.MessageEnabled(fpa::FpaOdomstatusPayload::MSG_NAME)) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Fusion status disabled");
        return;
    }

    bool have = false;
    fpa::FpaOdomstatusPayload payload;
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        have = have_odomstatus_;
        payload = last_odomstatus_;
    }

    if (!have) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No fusion status yet");
        return;
    }

    bool degraded = (payload.fusion_imu == fpa::FpaMeasStatus::DEGRADED) ||
                    (payload.fusion_gnss1 == fpa::FpaMeasStatus::DEGRADED) ||
                    (payload.fusion_gnss2 == fpa::FpaMeasStatus::DEGRADED) ||
                    (payload.fusion_corr == fpa::FpaMeasStatus::DEGRADED) ||
                    (payload.fusion_cam1 == fpa::FpaMeasStatus::DEGRADED) ||
                    (payload.fusion_ws == fpa::FpaMeasStatus::DEGRADED) ||
                    (payload.fusion_markers == fpa::FpaMeasStatus::DEGRADED);

    if (degraded) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Fusion degraded");
    } else if (payload.init_status != fpa::FpaInitStatus::GLOBAL_INIT) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Fusion not globally initialized");
    } else {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Fusion initialized");
    }

    status.add("init_status", static_cast<int>(payload.init_status));
    status.add("fusion_imu", static_cast<int>(payload.fusion_imu));
    status.add("fusion_gnss1", static_cast<int>(payload.fusion_gnss1));
    status.add("fusion_gnss2", static_cast<int>(payload.fusion_gnss2));
    status.add("fusion_corr", static_cast<int>(payload.fusion_corr));
    status.add("fusion_cam1", static_cast<int>(payload.fusion_cam1));
    status.add("fusion_ws", static_cast<int>(payload.fusion_ws));
    status.add("fusion_markers", static_cast<int>(payload.fusion_markers));
    status.add("imu_status", static_cast<int>(payload.imu_status));
    status.add("imu_noise", static_cast<int>(payload.imu_noise));
    status.add("imu_conv", static_cast<int>(payload.imu_conv));
    status.add("gnss1_status", static_cast<int>(payload.gnss1_status));
    status.add("gnss2_status", static_cast<int>(payload.gnss2_status));
    status.add("baseline_status", static_cast<int>(payload.baseline_status));
    status.add("corr_status", static_cast<int>(payload.corr_status));
    status.add("cam1_status", static_cast<int>(payload.cam1_status));
    status.add("ws_status", static_cast<int>(payload.ws_status));
    status.add("ws_conv", static_cast<int>(payload.ws_conv));
    status.add("markers_status", static_cast<int>(payload.markers_status));
    status.add("markers_conv", static_cast<int>(payload.markers_conv));
}

void FixpositionDriverNode::DiagnosticsGnss(diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (!params_.MessageEnabled(fpa::FpaGnsscorrPayload::MSG_NAME)) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "GNSS corrections disabled");
        return;
    }

    bool have = false;
    fpa::FpaGnsscorrPayload payload;
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        have = have_gnsscorr_;
        payload = last_gnsscorr_;
    }

    if (!have) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No GNSS correction status yet");
        return;
    }

    int gnss1_fix = static_cast<int>(payload.gnss1_fix);
    int gnss2_fix = static_cast<int>(payload.gnss2_fix);
    int best_fix = std::max(gnss1_fix, gnss2_fix);

    if (best_fix >= static_cast<int>(fpa::FpaGnssFix::RTK_FLOAT)) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "GNSS RTK fix");
    } else if (best_fix >= static_cast<int>(fpa::FpaGnssFix::S3D)) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "GNSS fix (non-RTK)");
    } else {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "GNSS no fix");
    }

    status.add("gnss1_fix", gnss1_fix);
    status.add("gnss2_fix", gnss2_fix);
    AddFpaInt(status, "gnss1_nsig_l1", payload.gnss1_nsig_l1);
    AddFpaInt(status, "gnss1_nsig_l2", payload.gnss1_nsig_l2);
    AddFpaInt(status, "gnss2_nsig_l1", payload.gnss2_nsig_l1);
    AddFpaInt(status, "gnss2_nsig_l2", payload.gnss2_nsig_l2);
    AddFpaFloat(status, "corr_latency_s", payload.corr_latency);
    AddFpaFloat(status, "corr_update_rate_hz", payload.corr_update_rate);
    AddFpaFloat(status, "corr_data_rate_kbps", payload.corr_data_rate);
    AddFpaFloat(status, "corr_msg_rate_hz", payload.corr_msg_rate);
    AddFpaInt(status, "sta_id", payload.sta_id);
    AddFpaInt(status, "sta_dist_m", payload.sta_dist);
}

void FixpositionDriverNode::DiagnosticsAntenna(diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (!params_.MessageEnabled(fpa::FpaGnssantPayload::MSG_NAME)) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "GNSS antenna diagnostics disabled");
        return;
    }

    bool have = false;
    fpa::FpaGnssantPayload payload;
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        have = have_gnssant_;
        payload = last_gnssant_;
    }

    if (!have) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No GNSS antenna status yet");
        return;
    }

    int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "Antenna OK";

    if ((payload.gnss1_state == fpa::FpaAntState::SHORT) || (payload.gnss2_state == fpa::FpaAntState::SHORT)) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        msg = "Antenna short";
    } else if ((payload.gnss1_state != fpa::FpaAntState::OK) || (payload.gnss2_state != fpa::FpaAntState::OK)) {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg = "Antenna not OK";
    }

    if ((payload.gnss1_power != fpa::FpaAntPower::ON) || (payload.gnss2_power != fpa::FpaAntPower::ON)) {
        if (level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
            level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            msg = "Antenna power off";
        }
    }

    status.summary(level, msg);
    status.add("gnss1_state", static_cast<int>(payload.gnss1_state));
    status.add("gnss1_power", static_cast<int>(payload.gnss1_power));
    AddFpaInt(status, "gnss1_age_s", payload.gnss1_age);
    status.add("gnss2_state", static_cast<int>(payload.gnss2_state));
    status.add("gnss2_power", static_cast<int>(payload.gnss2_power));
    AddFpaInt(status, "gnss2_age_s", payload.gnss2_age);
}

void FixpositionDriverNode::DiagnosticsText(diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (!params_.MessageEnabled(fpa::FpaTextPayload::MSG_NAME)) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Text diagnostics disabled");
        return;
    }

    bool have = false;
    fpa::FpaTextPayload payload;
    std::string text;
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        have = have_text_;
        payload = last_text_;
        text = last_text_string_;
    }

    if (!have) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No text messages");
        return;
    }

    int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (payload.level == fpa::FpaTextLevel::ERROR) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (payload.level == fpa::FpaTextLevel::WARNING) {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    status.summary(level, text.empty() ? "Text message received" : text);
    status.add("text_level", static_cast<int>(payload.level));
    status.add("text", text);
}

// Helper for advertising output topics
#define _PUB(_pub_, _type_, _topic_, ...)                                      \
    do {                                                                       \
        RCLCPP_INFO(logger_, "Advertise %s (" #_type_ ")", (_topic_).c_str()); \
        _pub_ = rclcpp::create_publisher<_type_>(node_interfaces_.parameters_, \
                                                 node_interfaces_.topics_, _topic_, __VA_ARGS__); \
    } while (0)

// Helper for subscribing to input topics
#define _SUB(_sub_, _type_, _topic_, ...)                                      \
    do {                                                                       \
        RCLCPP_INFO(logger_, "Subscribe %s (" #_type_ ")", (_topic_).c_str()); \
        _sub_ = rclcpp::create_subscription<_type_>(node_interfaces_.parameters_, \
                                                    node_interfaces_.topics_, _topic_, __VA_ARGS__); \
    } while (0)

bool FixpositionDriverNode::StartNode() {
    RCLCPP_INFO(logger_, "Starting...");

    driver_running_ = false;
    ResetDiagnosticsState();

    if (diagnostics_params_.enabled_) {
        diagnostics_updater_ = std::make_unique<diagnostic_updater::Updater>(
            node_interfaces_.base_, node_interfaces_.clock_, node_interfaces_.logging_,
            node_interfaces_.parameters_, node_interfaces_.timers_, node_interfaces_.topics_);
        diagnostics_updater_->setHardwareID(diagnostics_params_.hardware_id_);
        diagnostics_updater_->add("Driver", this, &FixpositionDriverNode::DiagnosticsDriver);
        diagnostics_updater_->add("Fusion", this, &FixpositionDriverNode::DiagnosticsFusion);
        diagnostics_updater_->add("GNSS Fix", this, &FixpositionDriverNode::DiagnosticsGnss);
        diagnostics_updater_->add("GNSS Antenna", this, &FixpositionDriverNode::DiagnosticsAntenna);
        diagnostics_updater_->add("FP Text", this, &FixpositionDriverNode::DiagnosticsText);

        auto period = std::chrono::duration<double>(1.0 / diagnostics_params_.rate_hz_);
        diagnostics_timer_ = rclcpp::create_wall_timer(
            period,
            [this]() {
                if (diagnostics_updater_) {
                    diagnostics_updater_->force_update();
                }
            },
            nullptr, node_interfaces_.base_.get(), node_interfaces_.timers_.get());

        driver_.AddRawObserver([this](const parser::ParserMsg& msg) {
            (void)msg;
            RecordRxTime();
        });
    }

    // TF
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_interfaces_.parameters_, node_interfaces_.topics_);
    static_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_interfaces_.parameters_,
                                                                       node_interfaces_.topics_);

    // Add observers and advertise output topics, depending on configuration
    const std::string output_ns = (params_.output_ns_.empty() ? node_interfaces_.base_->get_namespace()
                                                              : params_.output_ns_);

    // FP_A-ODOMETRY
    if (params_.MessageEnabled(fpa::FpaOdometryPayload::MSG_NAME)) {
        _PUB(fpa_odometry_pub_, fpmsgs::FpaOdometry, output_ns + "/fpa/odometry", qos_settings_);
        _PUB(odometry_ecef_pub_, nav_msgs::msg::Odometry, output_ns + "/odometry_ecef", qos_settings_);
        _PUB(odometry_llh_pub_, sensor_msgs::msg::NavSatFix, output_ns + "/odometry_llh", qos_settings_);
        _PUB(poiimu_pub_, sensor_msgs::msg::Imu, output_ns + "/poiimu", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaOdometryPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto odometry_payload = dynamic_cast<const fpa::FpaOdometryPayload&>(payload);
            PublishFpaOdometry(odometry_payload, fpa_odometry_pub_);
            PublishFpaOdometryDataImu(odometry_payload, params_.nav2_mode_, poiimu_pub_);
            PublishFpaOdometryDataNavSatFix(odometry_payload, params_.nav2_mode_, odometry_llh_pub_);
            OdometryData odometry_data;
            odometry_data.SetFromFpaOdomPayload(odometry_payload);
            PublishOdometryData(odometry_data, odometry_ecef_pub_);
            ProcessOdometryData(odometry_data);
            fusion_epoch_data_.CollectFpaOdometry(odometry_payload);
        });
    }

    // FP_A-ODOMSH
    if (params_.MessageEnabled(fpa::FpaOdomshPayload::MSG_NAME)) {
        _PUB(fpa_odomsh_pub_, fpmsgs::FpaOdomsh, output_ns + "/fpa/odomsh", qos_settings_);
        _PUB(odometry_smooth_pub_, nav_msgs::msg::Odometry, output_ns + "/odometry_smooth", qos_settings_);
        _PUB(odometry_enu_smooth_pub_, nav_msgs::msg::Odometry, output_ns + "/odometry_enu_smooth", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaOdomshPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto odomsh_payload = dynamic_cast<const fpa::FpaOdomshPayload&>(payload);
            PublishFpaOdomsh(odomsh_payload, fpa_odomsh_pub_);
            OdometryData odometry_data;
            odometry_data.SetFromFpaOdomPayload(odomsh_payload);

            // Update frames for Nav2
            if (params_.nav2_mode_) {
                odometry_data.frame_id = "odom";
                odometry_data.child_frame_id = "vrtk_link";
            }

            PublishOdometryData(odometry_data, odometry_smooth_pub_);
            ProcessOdometryData(odometry_data);
            fusion_epoch_data_.CollectFpaOdomsh(odomsh_payload);

            // Convert message to ENU
            if (ecef_enu0_tf_) {
                bool enu_valid = odometry_data.ConvertToEnu(*ecef_enu0_tf_);
                if (enu_valid) {
                    PublishOdometryData(odometry_data, odometry_enu_smooth_pub_);
                }
            }
        });
    }

    // FP_A-ODOMENU
    if (params_.MessageEnabled(fpa::FpaOdomenuPayload::MSG_NAME)) {
        _PUB(fpa_odomenu_pub_, fpmsgs::FpaOdomenu, output_ns + "/fpa/odomenu", qos_settings_);
        _PUB(odometry_enu_pub_, nav_msgs::msg::Odometry, output_ns + "/odometry_enu", qos_settings_);
        _PUB(eul_pub_, geometry_msgs::msg::Vector3Stamped, output_ns + "/ypr", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaOdomenuPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto odomenu_payload = dynamic_cast<const fpa::FpaOdomenuPayload&>(payload);
            PublishFpaOdomenu(odomenu_payload, fpa_odomenu_pub_);
            PublishFpaOdomenuVector3Stamped(odomenu_payload, eul_pub_);
            OdometryData odometry_data;
            odometry_data.SetFromFpaOdomPayload(odomenu_payload);

            // Update frames for Nav2
            if (params_.nav2_mode_) {
                odometry_data.frame_id = "map";
                odometry_data.child_frame_id = "vrtk_link";
            }

            PublishOdometryData(odometry_data, odometry_enu_pub_);
            ProcessOdometryData(odometry_data);
            fusion_epoch_data_.CollectFpaOdomenu(odomenu_payload);
        });
    }

    // FP_A-ODOMSTATUS
    if (params_.MessageEnabled(fpa::FpaOdomstatusPayload::MSG_NAME)) {
        _PUB(fpa_odomstatus_pub_, fpmsgs::FpaOdomstatus, output_ns + "/fpa/odomstatus", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaOdomstatusPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto odomstatus_payload = dynamic_cast<const fpa::FpaOdomstatusPayload&>(payload);
            StoreOdomstatus(odomstatus_payload);
            PublishFpaOdomstatus(odomstatus_payload, fpa_odomstatus_pub_);
            fusion_epoch_data_.CollectFpaOdomstatus(odomstatus_payload);
        });
    }

    // FP_A-EOE
    if (params_.MessageEnabled(fpa::FpaEoePayload::MSG_NAME)) {
        _PUB(fpa_eoe_pub_, fpmsgs::FpaEoe, output_ns + "/fpa/eoe", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaEoePayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto eoe_payload = dynamic_cast<const fpa::FpaEoePayload&>(payload);
            (void)eoe_payload;
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
        _PUB(eul_imu_pub_, geometry_msgs::msg::Vector3Stamped, output_ns + "/imu_ypr", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaTfPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            (void)payload;
            TfData tf;
            tf.SetFromFpaTfPayload(dynamic_cast<const fpa::FpaTfPayload&>(payload));
            ProcessTfData(tf);
        });
    }

    // FP_A-LLH
    if (params_.MessageEnabled(fpa::FpaLlhPayload::MSG_NAME)) {
        _PUB(fpa_llh_pub_, fpmsgs::FpaLlh, output_ns + "/fpa/llh", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaLlhPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaLlh(dynamic_cast<const fpa::FpaLlhPayload&>(payload), fpa_llh_pub_);
        });
    }

    // FP_A-GNSSANT
    if (params_.MessageEnabled(fpa::FpaGnssantPayload::MSG_NAME)) {
        _PUB(fpa_gnssant_pub_, fpmsgs::FpaGnssant, output_ns + "/fpa/gnssant", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaGnssantPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto gnssant_payload = dynamic_cast<const fpa::FpaGnssantPayload&>(payload);
            StoreGnssant(gnssant_payload);
            PublishFpaGnssant(gnssant_payload, fpa_gnssant_pub_);
        });
    }

    // FP_A-GNSSCORR
    if (params_.MessageEnabled(fpa::FpaGnsscorrPayload::MSG_NAME)) {
        _PUB(fpa_gnsscorr_pub_, fpmsgs::FpaGnsscorr, output_ns + "/fpa/gnsscorr", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaGnsscorrPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto gnsscorr_payload = dynamic_cast<const fpa::FpaGnsscorrPayload&>(payload);
            StoreGnsscorr(gnsscorr_payload);
            PublishFpaGnsscorr(gnsscorr_payload, fpa_gnsscorr_pub_);
        });
    }

    // FP_A-IMUBIAS
    if (params_.MessageEnabled(fpa::FpaImubiasPayload::MSG_NAME)) {
        _PUB(fpa_imubias_pub_, fpmsgs::FpaImubias, output_ns + "/fpa/imubias", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaImubiasPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto imubias_payload = dynamic_cast<const fpa::FpaImubiasPayload&>(payload);
            PublishFpaImubias(imubias_payload, fpa_imubias_pub_);
            fusion_epoch_data_.CollectFpaImubias(imubias_payload);
        });
    }

    // FP_A-RAWIMU
    if (params_.MessageEnabled(fpa::FpaRawimuPayload::MSG_NAME)) {
        _PUB(rawimu_pub_, fpmsgs::FpaImu, output_ns + "/fpa/rawimu", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaRawimuPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaRawimu(dynamic_cast<const fpa::FpaRawimuPayload&>(payload), rawimu_pub_);
        });
    }

    // FP_A-CORRIMU
    if (params_.MessageEnabled(fpa::FpaCorrimuPayload::MSG_NAME)) {
        _PUB(corrimu_pub_, fpmsgs::FpaImu, output_ns + "/fpa/corrimu", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaCorrimuPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaCorrimu(dynamic_cast<const fpa::FpaCorrimuPayload&>(payload), corrimu_pub_);
        });
    }

    // FP_A-TEXT
    if (params_.MessageEnabled(fpa::FpaTextPayload::MSG_NAME)) {
        _PUB(fpa_text_pub_, fpmsgs::FpaText, output_ns + "/fpa/text", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaTextPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            auto text_payload = dynamic_cast<const fpa::FpaTextPayload&>(payload);
            StoreText(text_payload);
            PublishFpaText(text_payload, fpa_text_pub_);
        });
    }

    // FP_A-TP
    if (params_.MessageEnabled(fpa::FpaTpPayload::MSG_NAME)) {
        _PUB(fpa_tp_pub_, fpmsgs::FpaTp, output_ns + "/fpa/tp", qos_settings_);
        driver_.AddFpaObserver(fpa::FpaTpPayload::MSG_NAME, [this](const fpa::FpaPayload& payload) {
            PublishFpaTp(dynamic_cast<const fpa::FpaTpPayload&>(payload), fpa_tp_pub_);
        });
    }

    // NOV_B-BESTGNSSPOS
    if (params_.MessageEnabled(novb::NOV_B_BESTGNSSPOS_STRID)) {
        _PUB(navsatfix_gnss1_pub_, sensor_msgs::msg::NavSatFix, output_ns + "/gnss1", qos_settings_);
        _PUB(navsatfix_gnss2_pub_, sensor_msgs::msg::NavSatFix, output_ns + "/gnss2", qos_settings_);
        driver_.AddNovbObserver(  //
            novb::NOV_B_BESTGNSSPOS_STRID, [this](const novb::NovbHeader* header, const uint8_t* payload) {
                if (!PublishNovbBestgnsspos(header, (novb::NovbBestgnsspos*)payload, navsatfix_gnss1_pub_,
                                            navsatfix_gnss2_pub_)) {
                    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1e3, "Bad NOV_B-BESTGNSSPOS");
                }
            });
    }

    // NOV_B-INSPVAX
    if (params_.MessageEnabled(novb::NOV_B_INSPVAX_STRID)) {
        _PUB(novb_inspvax_pub_, fpmsgs::NovbInspvax, output_ns + "/novb/inspvax", qos_settings_);
        driver_.AddNovbObserver(  //
            novb::NOV_B_INSPVAX_STRID, [this](const novb::NovbHeader* header, const uint8_t* payload) {
                if (!PublishNovbInspvax(header, (novb::NovbInspvax*)payload, novb_inspvax_pub_)) {
                    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1e3, "Bad NOV_B-INSPVAX");
                }
                fusion_epoch_data_.CollectNovbInspvax(header, (novb::NovbInspvax*)payload);
            });
    }

    // NOV_B-HEADING2
    if (params_.MessageEnabled(novb::NOV_B_HEADING2_STRID)) {
        _PUB(novb_heading2_pub_, fpmsgs::NovbHeading2, output_ns + "/novb/heading2", qos_settings_);
        driver_.AddNovbObserver(  //
            novb::NOV_B_HEADING2_STRID, [this](const novb::NovbHeader* header, const uint8_t* payload) {
                if (!PublishNovbHeading2(header, (novb::NovbHeading2*)payload, novb_heading2_pub_)) {
                    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1e3, "Bad NOV_B-HEADING2");
                }
            });
    }

    // NMEA-GP-GGA
    if (params_.MessageEnabled(nmea::NmeaGgaPayload::FORMATTER)) {
        _PUB(nmea_gga_pub_, fpmsgs::NmeaGga, output_ns + "/nmea/gga", qos_settings_);
        driver_.AddNmeaObserver(nmea::NmeaGgaPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gga_payload = dynamic_cast<const nmea::NmeaGgaPayload&>(payload);
            PublishNmeaGga(gga_payload, nmea_gga_pub_);
            nmea_epoch_data_.gga_ = gga_payload;
        });
    }

    // NMEA-GP-GLL
    if (params_.MessageEnabled(nmea::NmeaGllPayload::FORMATTER)) {
        _PUB(nmea_gll_pub_, fpmsgs::NmeaGll, output_ns + "/nmea/gll", qos_settings_);
        driver_.AddNmeaObserver(nmea::NmeaGllPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gll_payload = dynamic_cast<const nmea::NmeaGllPayload&>(payload);
            PublishNmeaGll(gll_payload, nmea_gll_pub_);
            nmea_epoch_data_.gll_ = gll_payload;
        });
    }

    // NMEA-GN-GSA
    if (params_.MessageEnabled(nmea::NmeaGsaPayload::FORMATTER)) {
        _PUB(nmea_gsa_pub_, fpmsgs::NmeaGsa, output_ns + "/nmea/gsa", qos_settings_);
        driver_.AddNmeaObserver(nmea::NmeaGsaPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gsa_payload_ = dynamic_cast<const nmea::NmeaGsaPayload&>(payload);
            PublishNmeaGsa(gsa_payload_, nmea_gsa_pub_);
            nmea_epoch_data_.gsa_ = gsa_payload_;
            nmea_epoch_data_.gsa_gsv_.AddGsa(gsa_payload_);
        });
    }

    // NMEA-GP-GST
    if (params_.MessageEnabled(nmea::NmeaGstPayload::FORMATTER)) {
        _PUB(nmea_gst_pub_, fpmsgs::NmeaGst, output_ns + "/nmea/gst", qos_settings_);
        driver_.AddNmeaObserver(nmea::NmeaGstPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gst_payload = dynamic_cast<const nmea::NmeaGstPayload&>(payload);
            PublishNmeaGst(gst_payload, nmea_gst_pub_);
            nmea_epoch_data_.gst_ = gst_payload;
        });
    }

    // NMEA-GX-GSV
    if (params_.MessageEnabled(nmea::NmeaGsvPayload::FORMATTER)) {
        _PUB(nmea_gsv_pub_, fpmsgs::NmeaGsv, output_ns + "/nmea/gsv", qos_settings_);
        driver_.AddNmeaObserver(nmea::NmeaGsvPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto gsv_payload_ = dynamic_cast<const nmea::NmeaGsvPayload&>(payload);
            PublishNmeaGsv(gsv_payload_, nmea_gsv_pub_);
            nmea_epoch_data_.gsa_gsv_.AddGsv(gsv_payload_);
        });
    }

    // NMEA-GP-HDT
    if (params_.MessageEnabled(nmea::NmeaHdtPayload::FORMATTER)) {
        _PUB(nmea_hdt_pub_, fpmsgs::NmeaHdt, output_ns + "/nmea/hdt", qos_settings_);
        driver_.AddNmeaObserver(nmea::NmeaHdtPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto hdt_payload = dynamic_cast<const nmea::NmeaHdtPayload&>(payload);
            PublishNmeaHdt(hdt_payload, nmea_hdt_pub_);
            nmea_epoch_data_.hdt_ = hdt_payload;
        });
    }

    // NMEA-GP-RMC
    if (params_.MessageEnabled(nmea::NmeaRmcPayload::FORMATTER)) {
        _PUB(nmea_rmc_pub_, fpmsgs::NmeaRmc, output_ns + "/nmea/rmc", qos_settings_);
        driver_.AddNmeaObserver(nmea::NmeaRmcPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto rmc_payload = dynamic_cast<const nmea::NmeaRmcPayload&>(payload);
            PublishNmeaRmc(rmc_payload, nmea_rmc_pub_);
            nmea_epoch_data_.rmc_ = rmc_payload;
        });
    }

    // NMEA-GP-VTG
    if (params_.MessageEnabled(nmea::NmeaVtgPayload::FORMATTER)) {
        _PUB(nmea_vtg_pub_, fpmsgs::NmeaVtg, output_ns + "/nmea/vtg", qos_settings_);
        driver_.AddNmeaObserver(nmea::NmeaVtgPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto vtg_payload = dynamic_cast<const nmea::NmeaVtgPayload&>(payload);
            PublishNmeaVtg(vtg_payload, nmea_vtg_pub_);
            nmea_epoch_data_.vtg_ = vtg_payload;
        });
    }

    // NMEA-GP-ZDA
    if (params_.MessageEnabled(nmea::NmeaZdaPayload::FORMATTER)) {
        _PUB(nmea_zda_pub_, fpmsgs::NmeaZda, output_ns + "/nmea/zda", qos_settings_);
        driver_.AddNmeaObserver(nmea::NmeaZdaPayload::FORMATTER, [this](const nmea::NmeaPayload& payload) {
            auto zda_payload = dynamic_cast<const nmea::NmeaZdaPayload&>(payload);
            PublishNmeaZda(zda_payload, nmea_zda_pub_);
            nmea_epoch_data_.zda_ = zda_payload;
        });
    }

    // Raw messages
    if (params_.raw_output_) {
        _PUB(raw_pub_, fpmsgs::ParserMsg, output_ns + "/raw", qos_settings_);
        driver_.AddRawObserver([this](const parser::ParserMsg& msg) { PublishParserMsg(msg, raw_pub_); });
    }

    // Fusion epoch
    if (params_.fusion_epoch_) {
        _PUB(fusion_epoch_pub_, fpmsgs::FusionEpoch, output_ns + "/fusion", qos_settings_);
        // Publish is triggered by FP_A-EOE above
    }

    // NMEA epoch
    if (params_.nmea_epoch_ != fpa::FpaEpoch::UNSPECIFIED) {
        _PUB(nmea_epoch_pub_, fpmsgs::NmeaEpoch, output_ns + "/nmea", qos_settings_);
        // Publish is triggered by FP_A-EOE above
    }

    // Jump warning message
    if (params_.cov_warning_) {
        _PUB(jump_pub_, fpmsgs::CovWarn, output_ns + "/extras/jump", qos_settings_);
    }

    // WGS84 datum message
    if (params_.nav2_mode_) {
        _PUB(datum_pub_, sensor_msgs::msg::NavSatFix, output_ns + "/datum", qos_settings_);
    }

    // Subscribe to correction data input
    if (!params_.corr_topic_.empty()) {
        _SUB(corr_sub_, rtcm_msgs::msg::Message, params_.corr_topic_, 100, [this](const rtcm_msgs::msg::Message& msg) {
            driver_.SendCorrectionData(msg.message.data(), msg.message.size());
        });
    }

    // Subscribe to wheelspeed input
    if (!params_.speed_topic_.empty()) {
        _SUB(ws_sub_, fpmsgs::Speed, params_.speed_topic_, 10,
             [this](const fpmsgs::Speed& msg) { driver_.SendWheelspeedData(SpeedMsgToWheelspeedData(msg)); });
    }

    // Subscribe to wheelspeed input
    if (params_.converter_enabled_) {
        if (!params_.converter_input_topic_.empty()) {
            switch (params_.converter_topic_type_) {
                case DriverParams::VelTopicType::TWIST:
                    _SUB(ws_conv_twist_sub_, geometry_msgs::msg::Twist, params_.converter_input_topic_, 10,
                         [this](const geometry_msgs::msg::Twist& msg) {
                             driver_.SendWheelspeedData(Vector3MsgToWheelspeedData(msg.linear, params_));
                         });
                    break;
                case DriverParams::VelTopicType::TWISTWITHCOV:
                    _SUB(ws_conv_twistcov_sub_, geometry_msgs::msg::TwistWithCovariance, params_.converter_input_topic_,
                         10, [this](const geometry_msgs::msg::TwistWithCovariance& msg) {
                             driver_.SendWheelspeedData(Vector3MsgToWheelspeedData(msg.twist.linear, params_));
                         });
                    break;
                case DriverParams::VelTopicType::ODOMETRY:
                    _SUB(ws_conv_odom_sub_, nav_msgs::msg::Odometry, params_.converter_input_topic_, 10,
                         [this](const nav_msgs::msg::Odometry& msg) {
                             driver_.SendWheelspeedData(Vector3MsgToWheelspeedData(msg.twist.twist.linear, params_));
                         });
                    break;
                default:
                    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1e3,
                                         "The selected wheelspeed input type is not supported!");
                    break;
            }
        }
    }

    bool started = driver_.StartDriver();
    driver_running_ = started;
    return started;
}

#undef _PUB
#undef _SUB

void FixpositionDriverNode::StopNode() {
    RCLCPP_INFO(logger_, "Stopping...");

    driver_.RemoveFpaObservers();
    driver_.RemoveNmeaObservers();
    driver_.RemoveNovbObservers();
    driver_.RemoveRawObservers();

    driver_.StopDriver();
    driver_running_ = false;

    diagnostics_timer_.reset();
    diagnostics_updater_.reset();
    ResetDiagnosticsState();

    // Stop advertising output topics
    // - FP_A messages
    fpa_eoe_pub_.reset();
    fpa_gnssant_pub_.reset();
    fpa_gnsscorr_pub_.reset();
    fpa_imubias_pub_.reset();
    fpa_llh_pub_.reset();
    fpa_odomenu_pub_.reset();
    fpa_odometry_pub_.reset();
    fpa_odomsh_pub_.reset();
    fpa_odomstatus_pub_.reset();
    fpa_text_pub_.reset();
    fpa_tp_pub_.reset();
    // - NMEA messages
    nmea_gga_pub_.reset();
    nmea_gll_pub_.reset();
    nmea_gsa_pub_.reset();
    nmea_gst_pub_.reset();
    nmea_gsv_pub_.reset();
    nmea_hdt_pub_.reset();
    nmea_rmc_pub_.reset();
    nmea_vtg_pub_.reset();
    nmea_zda_pub_.reset();
    // - NOV_B messages
    novb_inspvax_pub_.reset();
    novb_heading2_pub_.reset();
    // - Odometry
    odometry_ecef_pub_.reset();
    odometry_enu_pub_.reset();
    odometry_llh_pub_.reset();
    odometry_smooth_pub_.reset();
    odometry_enu_smooth_pub_.reset();
    // - Orientation
    eul_pub_.reset();
    eul_imu_pub_.reset();
    // - IMU
    corrimu_pub_.reset();
    poiimu_pub_.reset();
    rawimu_pub_.reset();
    // - GNSS
    navsatfix_gnss1_pub_.reset();
    navsatfix_gnss2_pub_.reset();
    fusion_epoch_pub_.reset();
    nmea_epoch_pub_.reset();
    // - Other
    jump_pub_.reset();
    raw_pub_.reset();
    datum_pub_.reset();

    // Stop input message subscribers
    ws_sub_.reset();
    corr_sub_.reset();
    ws_conv_twist_sub_.reset();
    ws_conv_twistcov_sub_.reset();
    ws_conv_odom_sub_.reset();

    // TF
    tf_br_.reset();
    static_br_.reset();
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::ProcessTfData(const TfData& tf_data) {
    // Check if TF is valid
    if (tf_data.rotation.w() == 0 && tf_data.rotation.vec().isZero()) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1e4,
                             "Invalid TF was found! Is the fusion engine initialized? Source: %s, target: %s",
                             tf_data.frame_id.c_str(), tf_data.child_frame_id.c_str());
        return;
    }

    // Generate TF message
    geometry_msgs::msg::TransformStamped tf;
    TfDataToTransformStamped(tf_data, tf);

    // TODO: use constants from helper.hpp?

    // FP_IMUH -> FP_POI
    if ((tf.child_frame_id == "FP_IMUH") && (tf.header.frame_id == "FP_POI")) {
        tf_br_->sendTransform(tf);

        // Publish pitch roll based on IMU only
        Eigen::Vector3d imu_ypr_eigen = trafo::QuatToEul(tf_data.rotation);
        imu_ypr_eigen.x() = 0.0;  // the yaw value is not observable using IMU alone
        geometry_msgs::msg::Vector3Stamped imu_ypr;
        imu_ypr.header.stamp = tf.header.stamp;
        imu_ypr.header.frame_id = "FP_POI";
        imu_ypr.vector.set__x(imu_ypr_eigen.x());
        imu_ypr.vector.set__y(imu_ypr_eigen.y());
        imu_ypr.vector.set__z(imu_ypr_eigen.z());
        eul_imu_pub_->publish(imu_ypr);
    }
    // FP_POI -> FP_POISH
    else if ((tf.child_frame_id == "FP_POISH") && (tf.header.frame_id == "FP_POI")) {
        tf_br_->sendTransform(tf);
        // Store  TF if Nav2 mode is enabled
        if (params_.nav2_mode_) {
            std::unique_lock<std::mutex> lock(tfs_.mutex_);
            tfs_.poi_poish_ = std::make_unique<geometry_msgs::msg::TransformStamped>(tf);
        }
    }
    // FP_ECEF -> FP_ENU0
    else if ((tf.child_frame_id == "FP_ENU0") && (tf.header.frame_id == "FP_ECEF")) {
        static_br_->sendTransform(tf);
        ecef_enu0_tf_ = std::make_unique<TfData>(tf_data);
        // Store TF if Nav2 mode is enabled
        if (params_.nav2_mode_) {
            std::unique_lock<std::mutex> lock(tfs_.mutex_);
            tfs_.ecef_enu0_ = std::make_unique<geometry_msgs::msg::TransformStamped>(tf);
        }
    }
    // Something else
    else {
        static_br_->sendTransform(tf);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void FixpositionDriverNode::ProcessOdometryData(const OdometryData& odometry_data) {
    // Send a warning if the system experiences delays
    // This message computes the difference between the message time and the local system time.
    // Thus, if the local time is off, the message might be triggered or not triggered when it should.
    if (params_.delay_warning_ > 0.0) {
        const double delay = (clock_->now() - fpsdk::ros2::utils::ConvTime(odometry_data.stamp)).seconds();
        if (delay > params_.delay_warning_) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1e3,
                                 "The system is experiencing significant delays! (estimated delay: %.3f seconds)",
                                 delay);
        }
    }

    switch (odometry_data.type) {
        case OdometryData::Type::ODOMETRY:

            if (odometry_data.valid) {
                geometry_msgs::msg::TransformStamped tf;
                OdometryDataToTransformStamped(odometry_data, tf);
                tf_br_->sendTransform(tf);
            }

            // Output jump warning
            if (params_.cov_warning_ && odometry_data.valid && jump_detector_.Check(odometry_data)) {
                RCLCPP_WARN(logger_, "%s", jump_detector_.warning_.c_str());
                PublishJumpWarning(jump_detector_, jump_pub_);
            }

            break;

        case OdometryData::Type::ODOMENU:
            // Store FP_ENU0 -> FP_POI TF if Nav2 mode is selected
            if (params_.nav2_mode_ && odometry_data.valid) {
                std::unique_lock<std::mutex> lock(tfs_.mutex_);
                tfs_.enu0_poi_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
                OdometryDataToTransformStamped(odometry_data, *tfs_.enu0_poi_);
            }
            break;

        case OdometryData::Type::ODOMSH:
            // Store TF if Nav2 mode is selected
            if (params_.nav2_mode_ && odometry_data.valid) {
                std::unique_lock<std::mutex> lock(tfs_.mutex_);
                tfs_.ecef_poish_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
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

    // Publish a static identity transform from FP_ENU0 to map
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = tfs_.ecef_enu0_->header.stamp;
    static_transform.header.frame_id = "FP_ENU0";
    static_transform.child_frame_id = "map";
    static_transform.transform.translation.x = 0.0;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_br_->sendTransform(static_transform);

    // Compute FP_ENU0 -> FP_POISH
    // Extract translation and rotation from ECEFENU0
    geometry_msgs::msg::Vector3 trans_ecef_enu0 = tfs_.ecef_enu0_->transform.translation;
    geometry_msgs::msg::Quaternion rot_ecef_enu0 = tfs_.ecef_enu0_->transform.rotation;
    Eigen::Vector3d t_ecef_enu0_;
    t_ecef_enu0_ << trans_ecef_enu0.x, trans_ecef_enu0.y, trans_ecef_enu0.z;
    Eigen::Quaterniond q_ecef_enu0_(rot_ecef_enu0.w, rot_ecef_enu0.x, rot_ecef_enu0.y, rot_ecef_enu0.z);

    // Extract translation and rotation from ECEFPOISH
    geometry_msgs::msg::Vector3 trans_ecef_poish = tfs_.ecef_poish_->transform.translation;
    geometry_msgs::msg::Quaternion rot_ecef_poish = tfs_.ecef_poish_->transform.rotation;
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
    geometry_msgs::msg::TransformStamped tfs_odom;
    tfs_odom.header.stamp = tfs_.enu0_poi_->header.stamp;
    tfs_odom.header.frame_id = "map";
    tfs_odom.child_frame_id = "odom";
    tfs_odom.transform = tf2::toMsg(tf_combined);
    tf_br_->sendTransform(tfs_odom);

    // Publish odom -> vrtk_link
    geometry_msgs::msg::TransformStamped tf_odom_base;
    tf_odom_base.header.stamp = tfs_.enu0_poi_->header.stamp;
    tf_odom_base.header.frame_id = "odom";
    tf_odom_base.child_frame_id = "vrtk_link";
    tf_odom_base.transform = tf2::toMsg(tf_ENU0POISH);
    tf_br_->sendTransform(tf_odom_base);

    // Publish WGS84 datum
    PublishDatum(trans_ecef_enu0, tfs_.enu0_poi_->header.stamp, datum_pub_);
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
