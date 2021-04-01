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
#include <fixposition_output/VRTK.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "base_converter.hpp"
#include "time_conversions.hpp"

#define FP_MSG_MAXLEN 400

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
     * $FP,gpsweek,gps_sec_in_week,x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,ax,ay,az,STATUS,pos_cov(6),orient_cov(6),vel_cov(6),version#XX
     *
     * @param state state message as string
     * @return nav_msgs::Odometry message
     */
    void convertAndPublish(const std::string& state, const ros::Publisher& odometry_pub, const ros::Publisher& imu_pub,
                           const ros::Publisher& navsat_pub, const ros::Publisher& status_pub) override;

    void handleFPMessage(const std::vector<std::string>& tokens, const ros::Publisher& odometry_pub,
                         const ros::Publisher& imu_pub, const ros::Publisher& status_pub);
    void handleLLHMessage(const std::vector<std::string>& tokens, const ros::Publisher& navsat_pub);

    static std::vector<double> buildCovMatrix12(double xx, double yy, double zz, double xy, double yz, double xz,
                                                double xx1, double yy1, double zz1, double xy1, double yz1, double xz1);
};

#endif  // FP_MSGCONVERTER