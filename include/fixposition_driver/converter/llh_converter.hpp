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
 * @date 2022-01-26
 *
 */
#ifndef __FIXPOSITION_DRIVER_CONVERTER_LLH_CONVERTER_HPP__
#define __FIXPOSITION_DRIVER_CONVERTER_LLH_CONVERTER_HPP__

/* SYSTEM / STL */

/* EXTERNAL */

/* ROS */
#include <sensor_msgs/NavSatFix.h>

/* PACKAGE */
#include <fixposition_driver/VRTK.h>

#include <fixposition_driver/converter/base_converter.hpp>
#include <fixposition_driver/time_conversions.hpp>

namespace fixposition {

class LlhConverter : public BaseConverter {
   public:
    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
    LlhConverter(ros::NodeHandle& nh)
        : navsatfix_pub_(nh.advertise<sensor_msgs::NavSatFix>("/fixposition/navsatfix", 100)) {}

    ~LlhConverter() = default;

    bool CheckHeader(const std::string msg_header) final { return msg_header == header_; }
    /**
     * @brief Comma Delimited FP Output msg
     * Example:
     * $FP,gpsweek,gps_sec_in_week,x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,ax,ay,az,STATUS,pos_cov(6),orient_cov(6),vel_cov(6),version#XX
     *
     * @param state state message as string
     * @return nav_msgs::Odometry message
     */
    void ConvertTokensAndPublish(const std::vector<std::string>& tokens) final;

   private:
    ros::Publisher navsatfix_pub_;
    const std::string header_ = "LLH";
};
}  // namespace fixposition
#endif  // FP_MSGCONVERTER