/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /
 *   /  /\  \
 *  /__/  \__\  Fixposition AG
 *
 * @file fp_msg_converter.hpp
 * @author Kailin Huang (kailin.huang@fixposition.com)
 * @brief
 * @date 2020-08-10
 *
 */
#ifndef FP_COMMON_ROS_OUTPUT_CONVERTER_FPMSGCONVERTER
#define FP_COMMON_ROS_OUTPUT_CONVERTER_FPMSGCONVERTER

#include "base_converter.hpp"
#include "time_conversions.hpp"

// ros msgs
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

class FpMsgConverter : public BaseConverter {
   public:
    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
    FpMsgConverter();

    /**
     * @brief Comma Delimited FP Output msg
     * Example:
     * $FP,gpsweek,gps_sec_in_week,x,y,z,vx,vy,vz,qw,qx,qy,qz,STATUS,#XX
     *
     * @param state state message as string
     * @return nav_msgs::Odometry message
     */
    nav_msgs::Odometry convert(const std::string& state) override;
};

#endif  // FP_COMMON_ROS_OUTPUT_CONVERTER_FPMSGCONVERTER