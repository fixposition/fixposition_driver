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
 * @brief Implementation of Parameter Loading
 */

/* LIBC/STL */
#include <cinttypes>

/* EXTERNAL */
#include <fpsdk_ros2/ext/rclcpp.hpp>

/* PACKAGE */
#include "fixposition_driver_ros2/params.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node>& nh, const std::string& ns, DriverParams& params) {
    auto logger = nh->get_logger();
    bool ok = true;
    RCLCPP_INFO(logger, "DriverParams: loading from %s", ns.c_str());

    const std::string STREAM = ns + ".stream";
    const std::string RECONNECT_DELAY = ns + ".reconnect_delay";
    const std::string MESSAGES = ns + ".messages";
    const std::string NMEA_EPOCH = ns + ".nmea_epoch";
    const std::string RAW_OUTPUT = ns + ".raw_output";
    const std::string COV_WARNING = ns + ".cov_warning";
    const std::string NAV2_MODE = ns + ".nav2_mode";

    nh->declare_parameter(STREAM, params.stream_);
    nh->declare_parameter(RECONNECT_DELAY, params.reconnect_delay_);
    nh->declare_parameter(MESSAGES, params.messages_);
    nh->declare_parameter(NMEA_EPOCH, "");
    nh->declare_parameter(RAW_OUTPUT, params.raw_output_);
    nh->declare_parameter(COV_WARNING, params.cov_warning_);
    nh->declare_parameter(NAV2_MODE, params.nav2_mode_);

    if (!nh->get_parameter(STREAM, params.stream_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", STREAM.c_str());
        ok = false;
    }
    if (!nh->get_parameter(RECONNECT_DELAY, params.reconnect_delay_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", RECONNECT_DELAY.c_str());
        ok = false;
    }
    if (!nh->get_parameter(MESSAGES, params.messages_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", MESSAGES.c_str());
        ok = false;
    }
    std::string epoch_str;
    if (!nh->get_parameter(NMEA_EPOCH, epoch_str)) {
        RCLCPP_WARN(logger, "Failed loading %s param", NMEA_EPOCH.c_str());
        ok = false;
    }
    if (!StrToEpoch(epoch_str, params.nmea_epoch_)) {
        RCLCPP_WARN(logger, "Bad value for %s param", NMEA_EPOCH.c_str());
        ok = false;
    }
    if (!nh->get_parameter(RAW_OUTPUT, params.raw_output_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", RAW_OUTPUT.c_str());
        ok = false;
    }
    if (!nh->get_parameter(COV_WARNING, params.cov_warning_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", COV_WARNING.c_str());
        ok = false;
    }
    if (!nh->get_parameter(NAV2_MODE, params.nav2_mode_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", NAV2_MODE.c_str());
        ok = false;
    }

    RCLCPP_INFO(logger, "DriverParams: stream=%s", params.stream_.c_str());
    RCLCPP_INFO(logger, "DriverParams: reconnect_delay=%.1f", params.reconnect_delay_);
    for (std::size_t ix = 0; ix < params.messages_.size(); ix++) {
        RCLCPP_INFO(logger, "DriverParams: messages[%" PRIuMAX "]=%s", ix, params.messages_[ix].c_str());
    }
    RCLCPP_INFO(logger, "DriverParams: nmea_epoch=%s", epoch_str.c_str());
    RCLCPP_INFO(logger, "DriverParams: raw_output=%s", params.raw_output_ ? "true" : "false");
    RCLCPP_INFO(logger, "DriverParams: cov_warning=%s", params.cov_warning_ ? "true" : "false");
    RCLCPP_INFO(logger, "DriverParams: nav2_mode=%s", params.nav2_mode_ ? "true" : "false");

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node>& nh, const std::string& ns, NodeParams& params) {
    auto logger = nh->get_logger();
    bool ok = true;
    RCLCPP_INFO(logger, "nhParams: loading from %s", ns.c_str());

    const std::string OUTPUT_NS = ns + ".output_ns";
    const std::string SPEED_TOPIC = ns + ".speed_topic";
    const std::string CORR_TOPIC = ns + ".corr_topic";
    const std::string QOS_TYPE = ns + ".qos_type";

    nh->declare_parameter(OUTPUT_NS, params.output_ns_);
    nh->declare_parameter(SPEED_TOPIC, params.speed_topic_);
    nh->declare_parameter(CORR_TOPIC, params.corr_topic_);
    nh->declare_parameter(QOS_TYPE, params.qos_type_);

    if (!nh->get_parameter(OUTPUT_NS, params.output_ns_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", OUTPUT_NS.c_str());
        ok = false;
    }
    if (!nh->get_parameter(SPEED_TOPIC, params.speed_topic_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", SPEED_TOPIC.c_str());
        ok = false;
    }
    if (!nh->get_parameter(CORR_TOPIC, params.corr_topic_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", CORR_TOPIC.c_str());
        ok = false;
    }
    if (!nh->get_parameter(QOS_TYPE, params.qos_type_)) {
        RCLCPP_WARN(logger, "Failed loading %s param", QOS_TYPE.c_str());
        ok = false;
    }

    RCLCPP_INFO(logger, "NodeParams: output_ns=%s", params.output_ns_.c_str());
    RCLCPP_INFO(logger, "NodeParams: speed_topic=%s", params.speed_topic_.c_str());
    RCLCPP_INFO(logger, "NodeParams: corr_topic=%s", params.corr_topic_.c_str());
    RCLCPP_INFO(logger, "NodeParams: qos_type=%s", params.qos_type_.c_str());

    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
