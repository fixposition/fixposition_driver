/**
 *  @file
 *  @brief Parameters
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

#ifndef __FIXPOSITION_DRIVER_ROS2_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_ROS2_PARAMS_HPP__

/* ROS */
#include <fixposition_driver_ros2/ros2_msgs.hpp>

/* FIXPOSITION */
#include <fixposition_driver_lib/params.hpp>

namespace fixposition {

/**
 * @brief Load all parameters from ROS parameter server
 *
 * @param[in] node
 * @param[in] ns namespace to load the parameters from
 * @param[out] params
 * @return true
 * @return false
 */
bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node> node, const std::string& ns, FpOutputParams& params);

/**
 * @brief Load all parameters from ROS parameter server
 *
 * @param[in] node
 * @param[in] ns namespace to load the parameters from
 * @param[out] params
 * @return true
 * @return false
 */

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node> node, const std::string& ns, CustomerInputParams& params);

/**
 * @brief Load all parameters from ROS parameter server
 *
 * @param[in] node
 * @param[out] params
 * @return true
 * @return false
 */
bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node> node, FixpositionDriverParams& params);

}  // namespace fixposition

#endif
