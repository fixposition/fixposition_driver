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

#ifndef __FIXPOSITION_DRIVER_ROS1_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_ROS1_PARAMS_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <fixposition_driver_lib/params.hpp>

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Load sensor parameters from rosparam server
 *
 * @param[in]  ns      Namespace to load the parameters from
 * @param[out] params  The sensor parameters
 *
 * @returns true on success, false otherwise
 */
bool LoadParamsFromRos1(const std::string& ns, DriverParams& params);

/**
 * @brief Load node parameters from rosparam server
 *
 * @param[in]  ns      Namespace to load the parameters from
 * @param[out] params  The sensor parameters
 *
 * @returns true on success, false otherwise
 */
bool LoadParamsFromRos1(const std::string& ns, NodeParams& params);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS1_PARAMS_HPP__
