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

#ifndef __FIXPOSITION_DRIVER_ROS2_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_ROS2_PARAMS_HPP__

/* LIBC/STL */
#include <memory>

/* EXTERNAL */
#include <fixposition_driver_lib/params.hpp>
#include <fpsdk_ros2/ext/rclcpp.hpp>

/* PACKAGE */

namespace rclcpp_lifecycle {
class LifecycleNode;
}  // namespace rclcpp_lifecycle

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Load sensor parameters from rosparam server
 *
 * @param[in]  nh      Node handle
 * @param[out] params  The sensor parameters
 *
 * @returns true on success, false otherwise
 */
template <typename NodeT>
bool LoadParamsFromRos2(const std::shared_ptr<NodeT>& nh, DriverParams& params);

/**
 * @brief Diagnostics configuration for ROS2 node
 */
struct DiagnosticsParams {
    bool enabled_ = false;
    double rate_hz_ = 1.0;
    int timeout_ms_ = 1000;
    std::string hardware_id_;
};

/**
 * @brief Load diagnostics parameters from rosparam server
 *
 * @param[in]  nh      Node handle
 * @param[out] params  The diagnostics parameters
 *
 * @returns true on success, false otherwise
 */
template <typename NodeT>
bool LoadDiagnosticsParamsFromRos2(const std::shared_ptr<NodeT>& nh, DiagnosticsParams& params);

extern template bool LoadParamsFromRos2<rclcpp::Node>(const std::shared_ptr<rclcpp::Node>& nh, DriverParams& params);
extern template bool LoadParamsFromRos2<rclcpp_lifecycle::LifecycleNode>(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& nh, DriverParams& params);
extern template bool LoadDiagnosticsParamsFromRos2<rclcpp::Node>(const std::shared_ptr<rclcpp::Node>& nh,
                                                                 DiagnosticsParams& params);
extern template bool LoadDiagnosticsParamsFromRos2<rclcpp_lifecycle::LifecycleNode>(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& nh, DiagnosticsParams& params);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS2_PARAMS_HPP__
