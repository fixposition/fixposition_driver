/**
 *  @file
 *  @brief Implementation of OdometryConverter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* EXTERNAL */
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/* ROS */
#include <geometry_msgs/TransformStamped.h>

/* FIXPOSITION */
#include <fixposition_gnss_tf/gnss_tf.hpp>

/* PACKAGE */
#include <fixposition_driver/converter/odometry.hpp>
#include <fixposition_driver/time_conversions.hpp>

namespace fixposition {

/// msg field indices
static constexpr const int msg_type_idx = 1;
static constexpr const int msg_version_idx = 2;
static constexpr const int gps_week_idx = 3;
static constexpr const int gps_tow_idx = 4;
static constexpr const int pos_x_idx = 5;
static constexpr const int pos_y_idx = 6;
static constexpr const int pos_z_idx = 7;
static constexpr const int orientation_w_idx = 8;
static constexpr const int orientation_x_idx = 9;
static constexpr const int orientation_y_idx = 10;
static constexpr const int orientation_z_idx = 11;
static constexpr const int vel_x_idx = 12;
static constexpr const int vel_y_idx = 13;
static constexpr const int vel_z_idx = 14;
static constexpr const int rot_x_idx = 15;
static constexpr const int rot_y_idx = 16;
static constexpr const int rot_z_idx = 17;
static constexpr const int acc_x_idx = 18;
static constexpr const int acc_y_idx = 19;
static constexpr const int acc_z_idx = 20;
static constexpr const int fusion_status_idx = 21;
static constexpr const int imu_bias_status_idx = 22;
static constexpr const int gnss_fix_type_idx = 23;
static constexpr const int wheelspeed_status_idx = 24;
static constexpr const int pos_cov_xx_idx = 25;
static constexpr const int pos_cov_yy_idx = 26;
static constexpr const int pos_cov_zz_idx = 27;
static constexpr const int pos_cov_xy_idx = 28;
static constexpr const int pos_cov_yz_idx = 29;
static constexpr const int pos_cov_xz_idx = 30;
static constexpr const int orientation_cov_xx_idx = 31;
static constexpr const int orientation_cov_yy_idx = 32;
static constexpr const int orientation_cov_zz_idx = 33;
static constexpr const int orientation_cov_xy_idx = 34;
static constexpr const int orientation_cov_yz_idx = 35;
static constexpr const int orientation_cov_xz_idx = 36;
static constexpr const int vel_cov_xx_idx = 37;
static constexpr const int vel_cov_yy_idx = 38;
static constexpr const int vel_cov_zz_idx = 39;
static constexpr const int vel_cov_xy_idx = 40;
static constexpr const int vel_cov_yz_idx = 41;
static constexpr const int vel_cov_xz_idx = 42;
static constexpr const int sw_version_idx = 43;

void OdometryConverter::ConvertTokensAndPublish(const std::vector<std::string>& tokens) {
    nav_msgs::Odometry odom_msg;
    sensor_msgs::Imu imu_msg;
    fixposition_driver::VRTK vrtk_msg;
    if (tokens.size() != 44) {
        ROS_INFO("Error in parsing Odometry string with %lu fields! odometry and status messages will be empty.",
                 tokens.size());
        return;
    }
    // Header & Stamps
    odom_msg.header.stamp = imu_msg.header.stamp = vrtk_msg.header.stamp =
        ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));

    odom_msg.header.frame_id = vrtk_msg.header.frame_id = "ECEF";
    odom_msg.child_frame_id = vrtk_msg.pose_frame = "POI";
    imu_msg.header.frame_id = vrtk_msg.kin_frame = "POI";

    // Pose & Cov
    Vector3ToMsg(odom_msg.pose.pose.position, tokens.at(pos_x_idx), tokens.at(pos_y_idx), tokens.at(pos_z_idx));
    Vector4ToMsg(odom_msg.pose.pose.orientation, tokens.at(orientation_w_idx), tokens.at(orientation_x_idx),
                 tokens.at(orientation_y_idx), tokens.at(orientation_z_idx));
    BuildCovMat6D(odom_msg.pose.covariance, StringToDouble(tokens.at(pos_cov_xx_idx)),
                  StringToDouble(tokens.at(pos_cov_yy_idx)), StringToDouble(tokens.at(pos_cov_zz_idx)),
                  StringToDouble(tokens.at(pos_cov_xy_idx)), StringToDouble(tokens.at(pos_cov_yz_idx)),
                  StringToDouble(tokens.at(pos_cov_xz_idx)), StringToDouble(tokens.at(orientation_cov_xx_idx)),
                  StringToDouble(tokens.at(orientation_cov_yy_idx)), StringToDouble(tokens.at(orientation_cov_zz_idx)),
                  StringToDouble(tokens.at(orientation_cov_xy_idx)), StringToDouble(tokens.at(orientation_cov_yz_idx)),
                  StringToDouble(tokens.at(orientation_cov_xz_idx)));
    vrtk_msg.pose = odom_msg.pose;

    // Twist & Cov
    // Linear
    Vector3ToMsg(odom_msg.twist.twist.linear, tokens.at(vel_x_idx), tokens.at(vel_y_idx), tokens.at(vel_z_idx));
    // Angular
    Vector3ToMsg(odom_msg.twist.twist.angular, tokens.at(rot_x_idx), tokens.at(rot_y_idx), tokens.at(rot_z_idx));

    BuildCovMat6D(odom_msg.twist.covariance, StringToDouble(tokens.at(vel_cov_xx_idx)),
                  StringToDouble(tokens.at(vel_cov_yy_idx)), StringToDouble(tokens.at(vel_cov_zz_idx)),
                  StringToDouble(tokens.at(vel_cov_xy_idx)), StringToDouble(tokens.at(vel_cov_yz_idx)),
                  StringToDouble(tokens.at(vel_cov_xz_idx)), 0, 0, 0, 0, 0, 0);
    vrtk_msg.velocity = odom_msg.twist;

    // POI IMU Message
    // Omega
    imu_msg.angular_velocity = odom_msg.twist.twist.angular;
    // Acceleration
    Vector3ToMsg(imu_msg.linear_acceleration, tokens.at(acc_x_idx), tokens.at(acc_y_idx), tokens.at(acc_z_idx));

    // Status
    vrtk_msg.fusion_status = tokens.at(fusion_status_idx).empty() ? 0 : tokens.at(fusion_status_idx)[0] - '0';
    vrtk_msg.imu_bias_status = tokens.at(imu_bias_status_idx).empty() ? 0 : tokens.at(imu_bias_status_idx)[0] - '0';
    vrtk_msg.gnss_status = tokens.at(gnss_fix_type_idx).empty() ? 0 : tokens.at(gnss_fix_type_idx)[0] - '0';
    vrtk_msg.wheelspeed_status =
        tokens.at(wheelspeed_status_idx).empty() ? 0 : tokens.at(wheelspeed_status_idx)[0] - '0';

    vrtk_msg.version = tokens.at(sw_version_idx).empty() ? "UNKNOWN" : tokens.at(sw_version_idx);

    //  TFs
    geometry_msgs::TransformStamped tf_ecef_poi, tf_ecef_enu;
    tf_ecef_enu.header.stamp = tf_ecef_poi.header.stamp = odom_msg.header.stamp;
    tf_ecef_enu.header.frame_id = tf_ecef_poi.header.frame_id = "ECEF";
    tf_ecef_poi.child_frame_id = "FP_POI";
    tf_ecef_enu.child_frame_id = "FP_ENU";  // The ENU frame at the position of FP_POI

    // TF ECEF POI is basically the same as the odometry, containing the Pose of the POI in the ECEF Frame
    PointToVector3(tf_ecef_poi.transform.translation, odom_msg.pose.pose.position);
    tf_ecef_poi.transform.rotation = odom_msg.pose.pose.orientation;

    // Quaternion from ECEF to Local ENU at POI
    const Eigen::Quaterniond q_enu_ecef = Eigen::Quaterniond(
        gnss_tf::RotEnuEcef(Eigen::Vector3d(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
                                            odom_msg.pose.pose.position.z))
            .transpose());
    // TF ECEF ENU
    PointToVector3(tf_ecef_enu.transform.translation, odom_msg.pose.pose.position);
    QuatToMsg(tf_ecef_enu.transform.rotation, q_enu_ecef);

    // Send Ros msgs
    if (odometry_pub_.getNumSubscribers() > 0) {
        odometry_pub_.publish(odom_msg);
    }
    if (imu_pub_.getNumSubscribers() > 0) {
        imu_pub_.publish(imu_msg);
    }
    if (vrtk_pub_.getNumSubscribers() > 0) {
        vrtk_pub_.publish(vrtk_msg);
    }

    // Send TFs
    if (CheckQuat(tf_ecef_poi.transform.rotation)) {
        br_.sendTransform(tf_ecef_poi);
    }
    if (CheckQuat(tf_ecef_enu.transform.rotation)) {
        br_.sendTransform(tf_ecef_enu);
    }
}

}  // namespace fixposition