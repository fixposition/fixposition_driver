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
 * \endverbatim
 *
 */

#ifndef __ODOM_CONVERTER_PARAMS_HPP__
#define __ODOM_CONVERTER_PARAMS_HPP__

/* SYSTEM / STL */
#include <string>

namespace fixposition {

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
     * @brief Load all parameters from ROS parameter server
     *
     * @param[in] ns namespace to load the parameters from
     * @return true success
     * @return false fail
     */
    bool LoadFromRos(const std::string& ns);
};

}  // namespace fixposition

#endif
