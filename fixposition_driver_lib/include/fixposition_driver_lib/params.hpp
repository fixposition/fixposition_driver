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

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Parameters for the sensor driver
 *
 * See launch/driver.yaml for documentation
 */
struct SensorParams {
    std::string stream_;
    double reconnect_delay_ = 5.0;
    std::vector<std::string> messages_;
    bool cov_warning_ = false;
    bool nav2_mode_ = false;

    // Check if entry is in messges_
    bool MessageEnabled(const std::string& message_name) const;

    // TODO remove
    std::vector<std::string> formats;  //!< Data formats (= messages) to process
};

/**
 * @brief Parameters for the ROS nodes
 */
struct NodeParams {
    std::string speed_topic_;  //!< Topic name for wheelspeed input
    std::string corr_topic_;   //!< Topic name for correction data input
    std::string qos_type_;     //!< ROS2 QoS type, supports "sensor_<short/long>" and "default_<short/long>"
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_PARAMS_HPP__
