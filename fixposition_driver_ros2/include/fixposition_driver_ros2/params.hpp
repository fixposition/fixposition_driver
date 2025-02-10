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

/* EXTERNAL */
#include <fixposition_driver_lib/params.hpp>
#include <fpsdk_ros2/ext/rclcpp.hpp>

/* PACKAGE */

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
bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node>& nh, DriverParams& params);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS2_PARAMS_HPP__
