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
#include <fpsdk_ros1/ext/ros.hpp>
#include <fpsdk_ros1/ext/ros_console.hpp>
#include <fpsdk_ros1/utils.hpp>

/* PACKAGE */
#include "fixposition_driver_ros1/params.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::ros1;

bool LoadParamsFromRos1(const std::string& ns, DriverParams& params) {
    bool ok = true;
    ROS_INFO("DriverParams: loading from %s", ns.c_str());

    if (!utils::LoadRosParam(ns + "/stream", params.stream_)) {
        ROS_WARN("Failed loading %s/stream param", ns.c_str());
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/reconnect_delay", params.reconnect_delay_)) {
        ROS_WARN("Failed loading %s/reconnect_delay param", ns.c_str());
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/messages", params.messages_)) {
        ROS_WARN("Failed loading %s/messages param", ns.c_str());
        ok = false;
    }
    std::string epoch_str;
    if (!utils::LoadRosParam(ns + "/nmea_epoch", epoch_str)) {
        ROS_WARN("Failed loading %s/nmea_epoch param", ns.c_str());
        ok = false;
    }
    if (!StrToEpoch(epoch_str, params.nmea_epoch_)) {
        ROS_WARN("Bad value for %s/nmea_epoch param", ns.c_str());
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/raw_output", params.raw_output_)) {
        ROS_WARN("Failed loading %s/raw_output param", ns.c_str());
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/cov_warning", params.cov_warning_)) {
        ROS_WARN("Failed loading %s/cov_warning param", ns.c_str());
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/nav2_mode", params.nav2_mode_)) {
        ROS_WARN("Failed loading %s/nav2_mode param", ns.c_str());
        ok = false;
    }

    ROS_INFO("DriverParams: stream=%s", params.stream_.c_str());
    ROS_INFO("DriverParams: reconnect_delay=%.1f", params.reconnect_delay_);
    for (std::size_t ix = 0; ix < params.messages_.size(); ix++) {
        ROS_INFO("DriverParams: messages[%" PRIuMAX "]=%s", ix, params.messages_[ix].c_str());
    }
    ROS_INFO("DriverParams: nmea_epoch=%s", epoch_str.c_str());
    ROS_INFO("DriverParams: raw_output=%s", params.raw_output_ ? "true" : "false");
    ROS_INFO("DriverParams: cov_warning=%s", params.cov_warning_ ? "true" : "false");
    ROS_INFO("DriverParams: nav2_mode=%s", params.nav2_mode_ ? "true" : "false");

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool LoadParamsFromRos1(const std::string& ns, NodeParams& params) {
    bool ok = true;
    ROS_INFO("NodeParams: loading from %s", ns.c_str());

    if (!utils::LoadRosParam(ns + "/output_ns", params.output_ns_)) {
        ROS_WARN("Failed loading %s/output_ns param", ns.c_str());
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/speed_topic", params.speed_topic_)) {
        ROS_WARN("Failed loading %s/speed_topic param", ns.c_str());
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/corr_topic", params.corr_topic_)) {
        ROS_WARN("Failed loading %s/corr_topic param", ns.c_str());
        ok = false;
    }

    ROS_INFO("NodeParams: output_ns=%s", params.output_ns_.c_str());
    ROS_INFO("NodeParams: speed_topic=%s", params.speed_topic_.c_str());
    ROS_INFO("NodeParams: corr_topic=%s", params.corr_topic_.c_str());

    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
