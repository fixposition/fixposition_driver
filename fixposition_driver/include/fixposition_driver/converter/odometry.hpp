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
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/* PACKAGE */
#include <fixposition_driver/msg/vrtk.hpp>

#include <fixposition_driver/converter/base_converter.hpp>
#include <fixposition_driver/time_conversions.hpp>

#include "geometry_msgs/msg/vector3.hpp"

namespace fixposition {

class OdometryConverter : public BaseConverter {
   public:
    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
    OdometryConverter(std::shared_ptr<rclcpp::Node> node)
        : BaseConverter(),
	  node_(node),
          tf_ecef_enu0_set_(false)
  {
          odometry_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry", 100);
          imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/fixposition/poiimu", 100);
          vrtk_pub_ = node_->create_publisher<fixposition_driver::msg::VRTK>("/fixposition/vrtk", 100);
          odometry_enu0_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry_enu", 100);
          eul_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("/fixposition/ypr", 100);
          eul_imu_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("/fixposition/imu_ypr", 100);

        tf_ecef_enu0_.header.frame_id = "ECEF";
        tf_ecef_enu0_.child_frame_id = "FP_ENU0";
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
	lt_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);
	br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);      
	static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
   }
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
    virtual void ConvertTokensAndPublish(const std::vector<std::string> &tokens) final;

   private:
    std::shared_ptr<rclcpp::Node> node_;
    const std::string header_ = "ODOMETRY";
    static constexpr const int kVersion_ = 1;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr     odometry_pub_;       //!< ECEF Odometry
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr       imu_pub_;            //!< Bias corrected IMU
    rclcpp::Publisher<fixposition_driver::msg::VRTK>::SharedPtr    vrtk_pub_;           //!< VRTK message
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr     odometry_enu0_pub_;  //!< ENU0 Odometry
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr eul_pub_;            //!< Euler angles Yaw-Pitch-Roll in local ENU
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr eul_imu_pub_;        //!< Euler angles Pitch-Roll as estimated from the IMU in local horizontal frame

    //! transform between ECEF and ENU0
    bool tf_ecef_enu0_set_;  //!< flag to indicate if the tf is already set
    Eigen::Vector3d t_ecef_enu0_;
    Eigen::Quaterniond q_ecef_enu0_;
    geometry_msgs::msg::TransformStamped tf_ecef_enu0_;  //!< tf from ECEF to ENU0

    std::shared_ptr<tf2_ros::Buffer>                     tf_buffer_;                      //!< tf buffer
    std::shared_ptr<tf2_ros::TransformListener>          lt_;                  //!< tf listener
    std::shared_ptr<tf2_ros::TransformBroadcaster>       br_;               //!< tf broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;  //!< tf_static broadcaster, for the ENU0 frame which is the ENU
                                                     //!< frame at the first received position
};

/**
 * @brief Build a 6x6 covariance matrix which is 2 independent 3x3 matrices
 *
 * [xx, xy, xz, 0, 0, 0,
 *  xy, yy, yz, 0, 0, 0,
 *  xz, yz, zz, 0, 0, 0,
 *  0, 0, 0, xx1, xy1, xz1,
 *  0, 0, 0, xy1, yy1, yz1,
 *  0, 0, 0, xz1, yz1, zz1]
 *
 * @param[in] xx
 * @param[in] yy
 * @param[in] zz
 * @param[in] xy
 * @param[in] yz
 * @param[in] xz
 * @param[in] xx1
 * @param[in] yy1
 * @param[in] zz1
 * @param[in] xy1
 * @param[in] yz1
 * @param[in] xz1
 * @return geometry_msgs::PoseWithCovariance::_covariance_type the 6x6 matrix
 */
inline geometry_msgs::msg::PoseWithCovariance::_covariance_type BuildCovMat6D(
    const double xx, const double yy, const double zz, const double xy, const double yz, const double xz, double xx1,
    const double yy1, const double zz1, const double xy1, const double yz1, double xz1) {
    geometry_msgs::msg::PoseWithCovariance::_covariance_type cov;
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

    return cov;
}

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_CONVERTER_ODOMETRY__
