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
 * @brief Parameters for the odometry converter
 */

#ifndef __FIXPOSITION_ODOMETRY_CONVERTER_ROS2_PARAMS_HPP__
#define __FIXPOSITION_ODOMETRY_CONVERTER_ROS2_PARAMS_HPP__

/* LIBC/STL */
#include <string>

/* EXTERNAL */

/* PACKAGE */
#include "fixposition_odometry_converter_ros2/ros2_msgs.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

enum class VelTopicType : int8_t { Twist = 0, TwistWithCov = 1, Odometry = 2 };

struct OdomInputParams {
    VelTopicType topic_type;
    std::string input_topic;
    std::string fixposition_speed_topic;
    int multiplicative_factor;
    bool use_x;
    bool use_y;
    bool use_z;
    /**
     * @brief Load all parameters from ROS 2
     *
     * @param[in] param_itf parameter interface
     * @param[in] logging_itf logging interface
     * @return true success
     * @return false fail
     */
    bool LoadFromRos(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& param_itf,
                     const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_itf);
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_ODOMETRY_CONVERTER_ROS2_PARAMS_HPP__
