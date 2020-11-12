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
#ifndef FP_MSGCONVERTER
#define FP_MSGCONVERTER

#include "base_converter.hpp"
#include "time_conversions.hpp"

#define FP_MSG_MAXLEN 200

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

#endif  // FP_MSGCONVERTER