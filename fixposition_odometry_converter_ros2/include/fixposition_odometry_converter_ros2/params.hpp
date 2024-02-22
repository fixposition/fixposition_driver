/**
 *  @file
 *  @brief Parameters for the odometry converter
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * 
 * Port to ROS 2 by Husarion
 * \endverbatim
 *
 */

#ifndef __ODOM_CONVERTER_PARAMS_HPP__
#define __ODOM_CONVERTER_PARAMS_HPP__

/* SYSTEM / STL */
#include <string>

/* ROS */
#include <rclcpp/rclcpp.hpp>

namespace fixposition {

enum class VelTopicType : int8_t { Twist = 0, TwistWithCov = 1, Odometry = 2 };

struct OdomInputParams {
    VelTopicType topic_type;
    std::string input_topic;
    std::string fixposition_speed_topic;
    int multiplicative_factor;
    int use_dimensions;
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

}  // namespace fixposition

#endif
