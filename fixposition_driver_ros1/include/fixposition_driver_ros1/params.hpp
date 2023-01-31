/**
 *  @file
 *  @brief Parameters
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_ROS1_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_ROS1_PARAMS_HPP__

/* SYSTEM / STL */

/* EXTERNAL */

/* FIXPOSITION */
#include <fixposition_driver_lib/params.hpp>

/* PACKAGE */

namespace fixposition {

/**
 * @brief Load all parameters from ROS parameter server
 *
 * @param[in] ns namespace to load the parameters from
 * @param[out] params
 * @return true
 * @return false
 */
bool LoadParamsFromRos1(const std::string& ns, FpOutputParams& params);

/**
 * @brief Load all parameters from ROS parameter server
 *
 * @param[in] ns namespace to load the parameters from
 * @param[out] params
 * @return true
 * @return false
 */
bool LoadParamsFromRos1(const std::string& ns, CustomerInputParams& params);

/**
 * @brief Load all parameters from ROS parameter server
 *
 * @param[in] ns namespace to load the parameters from
 * @param[out] params
 * @return true
 * @return false
 */
bool LoadParamsFromRos1(const std::string& ns, FixpositionDriverParams& params);

}  // namespace fixposition

#endif
