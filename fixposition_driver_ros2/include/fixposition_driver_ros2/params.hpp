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

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Load all parameters from ROS parameter server
 *
 * @param[in]  node    Node handle
 * @param[in]  ns      Namespace to load the parameters from
 * @param[out] params  The parameters
 *
 * @returns true on success, false otherwise
 */
bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node>& node, const std::string& ns, FpOutputParams& params);

/**
 * @brief Load all parameters from ROS parameter server
 *
 * @param[in]  node    Node handle
 * @param[in]  ns      Namespace to load the parameters from
 * @param[out] params  The parameters
 *
 * @returns true on success, false otherwise
 */

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node>& node, const std::string& ns, CustomerInputParams& params);

/**
 * @brief Load all parameters from ROS parameter server
 *
 * @param[in]  node    Node handle
 * @param[in]  ns      Namespace to load the parameters from
 * @param[out] params  The parameters
 *
 * @returns true on success, false otherwise
 */
bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node>& node, FixpositionDriverParams& params);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS2_PARAMS_HPP__
