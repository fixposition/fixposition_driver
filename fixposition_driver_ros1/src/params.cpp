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
#include <fixposition_driver_ros1/params.hpp>
#include <fpsdk_ros1/ext/ros.hpp>
#include <fpsdk_ros1/ext/ros_console.hpp>
#include <fpsdk_ros1/utils.hpp>

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::ros1;

bool LoadParamsFromRos1(const std::string& ns, SensorParams& params) {
    bool ok = true;

    if (!utils::LoadRosParam(ns + "/sensor/stream", params.stream_)) {
        ROS_WARN("Failed loading /sensor/stream param");
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/sensor/reconnect_delay", params.reconnect_delay_)) {
        ROS_WARN("Failed loading /sensor/reconnect_delay param");
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/sensor/messages", params.messages_)) {
        ROS_WARN("Failed loading /sensor/messages param");
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/sensor/cov_warning", params.cov_warning_)) {
        ROS_WARN("Failed loading /sensor/cov_warning param");
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/sensor/nav2_mode", params.nav2_mode_)) {
        ROS_WARN("Failed loading /sensor/nav2_mode param");
        ok = false;
    }

    ROS_INFO("SensorParams: stream=%s", params.stream_.c_str());
    ROS_INFO("SensorParams: reconnect_delay=%.1f", params.reconnect_delay_);
    for (std::size_t ix = 0; ix < params.messages_.size(); ix++) {
        ROS_INFO("SensorParams: messages[%" PRIuMAX "]=%s", ix, params.messages_[ix].c_str());
    }
    ROS_INFO("SensorParams: cov_warning=%s", params.cov_warning_ ? "true" : "false");
    ROS_INFO("SensorParams: nav2_mode=%s", params.nav2_mode_ ? "true" : "false");

    // TODO remove
    if (!utils::LoadRosParam(ns + "/sensor/formats", params.formats)) {
        ROS_WARN("Failed loading /sensor/formats param");
        ok = false;
    }
    for (size_t i = 0; i < params.formats.size(); i++) {
        ROS_INFO("formats[%ld] : %s", i, params.formats[i].c_str());
    }

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool LoadParamsFromRos1(const std::string& ns, NodeParams& params) {
    bool ok = true;

    if (!utils::LoadRosParam(ns + "/node/speed_topic", params.speed_topic_)) {
        ROS_WARN("Failed loading /node/speed_topic param");
        ok = false;
    }
    if (!utils::LoadRosParam(ns + "/node/corr_topic", params.corr_topic_)) {
        ROS_WARN("Failed loading /node/corr_topic param");
        ok = false;
    }

    ROS_INFO("NodeParams: speed_topic=%s", params.speed_topic_.c_str());
    ROS_INFO("NodeParams: corr_topic=%s", params.corr_topic_.c_str());

    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
