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
 * @brief Parameters
 */

#ifndef __FIXPOSITION_DRIVER_LIB_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_LIB_PARAMS_HPP__

/* LIBC/STL */
#include <string>
#include <vector>

/* EXTERNAL */
#include <fpsdk_common/parser/fpa.hpp>

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Enum with input types for the wheelspeed converter
 *
 * See launch/config.yaml for documentation
 */
enum class VelTopicType : int8_t { TWIST = 0, TWISTWITHCOV = 1, ODOMETRY = 2, UNSPECIFIED = -1 };

/**
 * @brief Parameters for the sensor driver
 *
 * See launch/config.yaml for documentation
 */
struct DriverParams {
    std::string stream_;
    double reconnect_delay_ = 5.0;
    std::vector<std::string> messages_;
    fpsdk::common::parser::fpa::FpaEpoch nmea_epoch_ = fpsdk::common::parser::fpa::FpaEpoch::UNSPECIFIED;
    bool raw_output_ = false;
    bool cov_warning_ = false;
    bool nav2_mode_ = false;

    bool converter_enabled_ = false;
    VelTopicType converter_topic_type_ = VelTopicType::UNSPECIFIED;
    std::string converter_input_topic_;
    double converter_scale_factor_ = 1.0;
    bool converter_use_x_ = false;
    bool converter_use_y_ = false;
    bool converter_use_z_ = false;

    // Check if entry is in messges_
    bool MessageEnabled(const std::string& message_name) const;
};

/**
 * @brief Parameters for the ROS nodes
 *
 * See launch/config.yaml for documentation
 */
struct NodeParams {
    std::string output_ns_;
    std::string speed_topic_;
    std::string corr_topic_;
    std::string qos_type_;
};

bool StrToEpoch(const std::string& str, fpsdk::common::parser::fpa::FpaEpoch& epoch);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_PARAMS_HPP__
