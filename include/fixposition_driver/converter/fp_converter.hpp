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
#ifndef __FIXPOSITION_DRIVER_CONVERTER_FP_CONVERTER_HPP__
#define __FIXPOSITION_DRIVER_CONVERTER_FP_CONVERTER_HPP__

/* SYSTEM / STL */

/* EXTERNAL */

/* ROS */
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

/* PACKAGE */
#include <fixposition_driver/VRTK.h>

#include <fixposition_driver/converter/base_converter.hpp>
#include <fixposition_driver/time_conversions.hpp>
namespace fixposition {

class FpConverter : public BaseConverter {
   public:
    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
    FpConverter(ros::NodeHandle& nh)
        : odometry_pub_(nh.advertise<nav_msgs::Odometry>("/fixposition/odometry", 100)),
          imu_pub_(nh.advertise<sensor_msgs::Imu>("/fixposition/imu", 100)),
          status_pub_(nh.advertise<fixposition_driver::VRTK>("/fixposition/vrtk", 100)),
          BaseConverter() {}

    ~FpConverter() = default;

    bool CheckHeader(const std::string msg_header) final { return msg_header == header_; }

    /**
     * @brief Comma Delimited FP Output msg
     * Example:
     * $FP,gpsweek,gps_sec_in_week,x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,ax,ay,az,STATUS,pos_cov(6),orient_cov(6),vel_cov(6),version#XX
     *
     * @param state state message as string
     * @return nav_msgs::Odometry message
     */
    virtual void ConvertTokensAndPublish(const std::vector<std::string>& tokens) final;

    /**
     * @brief Build a 6x6 covariance matrix which is 2 independent 3x4 matrices
     *
     * [xx0, xy0, xz0, 0, 0, 0,
     *  xy0, yy0, yz0, 0, 0, 0,
     *  xz0, yz0, zz0, 0, 0, 0,
     *  0, 0, 0, xx1, xy1, xz1,
     *  0, 0, 0, xy1, yy1, yz1,
     *  0, 0, 0, xz1, yz1, zz1]
     *
     * @param[in] xx0
     * @param[in] yy0
     * @param[in] zz0
     * @param[in] xy0
     * @param[in] yz0
     * @param[in] xz0
     * @param[in] xx1
     * @param[in] yy1
     * @param[in] zz1
     * @param[in] xy1
     * @param[in] yz1
     * @param[in] xz1
     * @return std::vector<double> the 6x6 matrix in row major 1D representation
     */
    static std::vector<double> BuildCovMat6D(double xx0, double yy0, double zz0, double xy0, double yz0, double xz0,
                                             double xx1, double yy1, double zz1, double xy1, double yz1, double xz1);

   private:
    ros::Publisher odometry_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher status_pub_;
    const std::string header_ = "FP";
};
}  // namespace fixposition
#endif  // FP_MSGCONVERTER