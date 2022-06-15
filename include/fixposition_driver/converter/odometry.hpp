/**
 *  @file
 *  @brief Declaration of OdometryConverter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_CONVERTER_ODOMETRY__
#define __FIXPOSITION_DRIVER_CONVERTER_ODOMETRY__

/* SYSTEM / STL */

/* EXTERNAL */

/* ROS */
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>

/* PACKAGE */
#include <fixposition_driver/VRTK.h>

#include <fixposition_driver/converter/base_converter.hpp>
#include <fixposition_driver/time_conversions.hpp>

namespace fixposition {

class OdometryConverter : public BaseConverter {
   public:
    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
    OdometryConverter(ros::NodeHandle& nh)
        : BaseConverter(),
          odometry_pub_(nh.advertise<nav_msgs::Odometry>("/fixposition/odometry", 100)),
          imu_pub_(nh.advertise<sensor_msgs::Imu>("/fixposition/poiimu", 100)),
          vrtk_pub_(nh.advertise<fixposition_driver::VRTK>("/fixposition/vrtk", 100)) {}

    ~OdometryConverter() = default;

    bool CheckHeaderAndVersion(const std::string msg_header, const std::string msg_version) final {
        return msg_header == header_ && std::stoi(msg_version) == kVersion_;
    }

    /**
     * @brief Comma Delimited FP Output msg
     * Example:
     * $FP,gpsweek,gps_sec_in_week,x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,ax,ay,az,STATUS,pos_cov(6),orient_cov(6),vel_cov(6),version#XX
     *
     * @param[in] state state message as string
     * @return nav_msgs::Odometry message
     */
    virtual void ConvertTokensAndPublish(const std::vector<std::string>& tokens) final;

   private:
    const std::string header_ = "ODOMETRY";
    static constexpr const int kVersion_ = 1;
    tf2_ros::TransformBroadcaster br_;
    ros::Publisher odometry_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher vrtk_pub_;
};

/**
 * @brief Build a 6x6 covariance matrix which is 2 independent 3x3 matrices
 *
 * [xx0, xy0, xz0, 0, 0, 0,
 *  xy0, yy0, yz0, 0, 0, 0,
 *  xz0, yz0, zz0, 0, 0, 0,
 *  0, 0, 0, xx1, xy1, xz1,
 *  0, 0, 0, xy1, yy1, yz1,
 *  0, 0, 0, xz1, yz1, zz1]
 *
 * @param[out] cov
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
inline void BuildCovMat6D(geometry_msgs::PoseWithCovariance::_covariance_type& cov, const double xx, const double yy,
                          const double zz, const double xy, const double yz, const double xz, double xx1,
                          const double yy1, const double zz1, const double xy1, const double yz1, double xz1) {
    // Diagonals
    cov[0 + 6 * 0] = xx;   // 0
    cov[1 + 6 * 1] = yy;   // 7
    cov[2 + 6 * 2] = zz;   // 14
    cov[3 + 6 * 3] = xx1;  // 21
    cov[4 + 6 * 4] = yy1;  // 28
    cov[5 + 6 * 5] = zz1;  // 35

    // Rest of values
    cov[1 + 6 * 0] = cov[0 + 6 * 1] = xy;   // 1 = 6
    cov[2 + 6 * 1] = cov[1 + 6 * 2] = yz;   // 8 = 13
    cov[2 + 6 * 0] = cov[0 + 6 * 2] = xz;   // 2 = 12
    cov[4 + 6 * 3] = cov[3 + 6 * 4] = xy1;  // 22 = 27
    cov[5 + 6 * 4] = cov[4 + 6 * 5] = yz1;  // 29 = 34
    cov[5 + 6 * 3] = cov[3 + 6 * 5] = xz1;  // 23 = 33
}

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_CONVERTER_ODOMETRY__
