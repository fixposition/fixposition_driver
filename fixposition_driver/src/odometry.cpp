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

/**
 * @brief Parse status flag field
 * 
 * @param[in] tokens list of tokens
 * @param[in] idx status flag index
 * @return int 
 */
int ParseStatusFlag(const std::vector<std::string> &tokens, const int idx) {
    if (tokens.at(idx).empty()) {
        return -1;
    } else {
        return std::stoi(tokens.at(idx));
    }
}

void OdometryConverter::ConvertTokensAndPublish(const std::vector<std::string> &tokens) {
    if (tokens.size() != 44) {
        ROS_INFO("Error in parsing Odometry string with %lu fields! odometry and status messages will be empty.",
                 tokens.size());
        return;
    }
    const int fusion_status = ParseStatusFlag(tokens, fusion_status_idx);

    const bool fusion_init = fusion_status >= 3;

    const bool odom_sub_avl = odometry_pub_.getNumSubscribers() > 0;
    const bool vrtk_sub_avl = vrtk_pub_.getNumSubscribers() > 0;
    const bool imu_sub_avl = imu_pub_.getNumSubscribers() > 0;
    const bool eul_sub_avl = eul_pub_.getNumSubscribers() > 0;
    const bool eul_imu_sub_avl = eul_imu_pub_.getNumSubscribers() > 0;
    const bool odom_enu0_sub_avl = odometry_enu0_pub_.getNumSubscribers() > 0;

    // common data
    const auto stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));
    const Eigen::Vector3d t_ecef_body =
        Vector3ToEigen(tokens.at(pos_x_idx), tokens.at(pos_y_idx), tokens.at(pos_z_idx));
    const Eigen::Quaterniond q_ecef_body = Vector4ToEigen(tokens.at(orientation_w_idx), tokens.at(orientation_x_idx),
                                                          tokens.at(orientation_y_idx), tokens.at(orientation_z_idx));
    if (fusion_init) {
        //  TFs
        geometry_msgs::TransformStamped tf_ecef_poi, tf_ecef_enu;
        tf_ecef_enu.header.stamp = stamp;
        tf_ecef_poi.header.stamp = stamp;
        tf_ecef_enu.header.frame_id = "ECEF";
        tf_ecef_poi.header.frame_id = "ECEF";
        tf_ecef_poi.child_frame_id = "FP_POI";
        tf_ecef_enu.child_frame_id = "FP_ENU";  // The ENU frame at the position of FP_POI

        // static TF ECEF ENU0
        if (!tf_ecef_enu0_set_ && tf_ecef_enu0_.transform.translation.x == 0 &&
            tf_ecef_enu0_.transform.translation.y == 0 && tf_ecef_enu0_.transform.translation.z == 0) {
            // ENU0 frame is not yet set, set the same ENU tf to ENU0
            t_ecef_enu0_ = t_ecef_body;
            q_ecef_enu0_ = Eigen::Quaterniond(gnss_tf::RotEnuEcef(t_ecef_body).transpose());
            tf_ecef_enu0_.transform.translation = EigenToVector3Msg(t_ecef_enu0_);
            tf_ecef_enu0_.transform.rotation = EigenToQuatMsg(q_ecef_enu0_);
            tf_ecef_enu0_set_ = true;
        }

        // TF ECEF POI is basically the same as the odometry, containing the Pose of the POI in the ECEF Frame
        tf_ecef_poi.transform.translation = EigenToVector3Msg(t_ecef_body);
        tf_ecef_poi.transform.rotation = EigenToQuatMsg(q_ecef_body);

        // Quaternion from ECEF to Local ENU at POI
        const Eigen::Quaterniond q_ecef_enu = Eigen::Quaterniond(gnss_tf::RotEnuEcef(t_ecef_body).transpose());
        // TF ECEF ENU
        tf_ecef_enu.transform.translation = EigenToVector3Msg(t_ecef_body);
        tf_ecef_enu.transform.rotation = EigenToQuatMsg(q_ecef_enu);

        // Send TFs
        if (CheckQuat(tf_ecef_poi.transform.rotation)) {
            br_.sendTransform(tf_ecef_poi);
        }
        if (CheckQuat(tf_ecef_enu.transform.rotation)) {
            br_.sendTransform(tf_ecef_enu);
        }
        // Send Static TF ECEF ENU0
        if (tf_ecef_enu0_set_ && CheckQuat(tf_ecef_enu0_.transform.rotation)) {
            static_br_.sendTransform(tf_ecef_enu0_);
        }
    }
    // Msgs
    nav_msgs::Odometry odom_msg;  //!<  Odmetry msg ECEF - FP_POI
    fixposition_driver::VRTK vrtk_msg;
    if (fusion_init) {
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "ECEF";
        odom_msg.child_frame_id = "FP_POI";

        vrtk_msg.header.stamp = stamp;
        vrtk_msg.header.frame_id = "ECEF";
        vrtk_msg.pose_frame = "FP_POI";
        vrtk_msg.kin_frame = "FP_POI";

        // Pose & Cov
        odom_msg.pose.pose.position = EigenToPointMsg(t_ecef_body);
        odom_msg.pose.pose.orientation = EigenToQuatMsg(q_ecef_body);
        odom_msg.pose.covariance = BuildCovMat6D(
            StringToDouble(tokens.at(pos_cov_xx_idx)), StringToDouble(tokens.at(pos_cov_yy_idx)),
            StringToDouble(tokens.at(pos_cov_zz_idx)), StringToDouble(tokens.at(pos_cov_xy_idx)),
            StringToDouble(tokens.at(pos_cov_yz_idx)), StringToDouble(tokens.at(pos_cov_xz_idx)),
            StringToDouble(tokens.at(orientation_cov_xx_idx)), StringToDouble(tokens.at(orientation_cov_yy_idx)),
            StringToDouble(tokens.at(orientation_cov_zz_idx)), StringToDouble(tokens.at(orientation_cov_xy_idx)),
            StringToDouble(tokens.at(orientation_cov_yz_idx)), StringToDouble(tokens.at(orientation_cov_xz_idx)));
        vrtk_msg.pose = odom_msg.pose;

        // Twist & Cov
        // Linear
        odom_msg.twist.twist.linear = Vector3ToMsg(tokens.at(vel_x_idx), tokens.at(vel_y_idx), tokens.at(vel_z_idx));
        // Angular
        odom_msg.twist.twist.angular = Vector3ToMsg(tokens.at(rot_x_idx), tokens.at(rot_y_idx), tokens.at(rot_z_idx));
        odom_msg.twist.covariance = BuildCovMat6D(
            StringToDouble(tokens.at(vel_cov_xx_idx)), StringToDouble(tokens.at(vel_cov_yy_idx)),
            StringToDouble(tokens.at(vel_cov_zz_idx)), StringToDouble(tokens.at(vel_cov_xy_idx)),
            StringToDouble(tokens.at(vel_cov_yz_idx)), StringToDouble(tokens.at(vel_cov_xz_idx)), 0, 0, 0, 0, 0, 0);
        vrtk_msg.velocity = odom_msg.twist;
    }

    // Status, regardless of fusion_init
    vrtk_msg.fusion_status = fusion_status;
    vrtk_msg.imu_bias_status = ParseStatusFlag(tokens, imu_bias_status_idx);
    vrtk_msg.gnss_status = ParseStatusFlag(tokens, gnss_fix_type_idx);
    vrtk_msg.wheelspeed_status = ParseStatusFlag(tokens, wheelspeed_status_idx);

    vrtk_msg.version = tokens.at(sw_version_idx).empty() ? "UNKNOWN" : tokens.at(sw_version_idx);

    // Send Ros msgs
    if (odom_sub_avl && fusion_init) {
        odometry_pub_.publish(odom_msg);
    }
    if (vrtk_sub_avl) {
        vrtk_pub_.publish(vrtk_msg);
    }
    if (imu_sub_avl) {
        // POI IMU Message
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = "FP_POI";
        // Omega
        imu_msg.angular_velocity = odom_msg.twist.twist.angular;
        // Acceleration
        imu_msg.linear_acceleration = Vector3ToMsg(tokens.at(acc_x_idx), tokens.at(acc_y_idx), tokens.at(acc_z_idx));
        imu_pub_.publish(imu_msg);
    }
    if (eul_sub_avl && fusion_init) {
        // Euler angle wrt. ENU frame in the order of Yaw Pitch Roll
        const Eigen::Vector3d ypr = gnss_tf::EcefPoseToEnuEul(t_ecef_body, q_ecef_body.toRotationMatrix());
        eul_pub_.publish(EigenToVector3Msg(ypr));
    }
    if (eul_imu_sub_avl) {
        bool valid = true;
        geometry_msgs::TransformStamped imu_pose;
        try {
            // Check if the transform is being correctly published
            imu_pose = tf_buffer_.lookupTransform("FP_IMU_HORIZONTAL", "FP_POI", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            valid = false;
        }
        if (valid) {
            const Eigen::Quaterniond quat_pose = QuatMsgToEigen(imu_pose.transform.rotation);
            Eigen::Vector3d imu_ypr = gnss_tf::QuatToEul(quat_pose);
            imu_ypr.x() = 0.0;  // the yaw value is not observable using IMU alone
            eul_imu_pub_.publish(EigenToVector3Msg(imu_ypr));
        }
    }
    if (odom_enu0_sub_avl && fusion_init) {
        // Odmetry msg ENU0 - FP_POI
        nav_msgs::Odometry odom_enu0_msg;
        odom_enu0_msg.header.stamp = stamp;
        odom_enu0_msg.header.frame_id = "ENU0";
        odom_enu0_msg.child_frame_id = "FP_POI";
        // Pose
        // convert position in ECEF into position in ENU0
        const Eigen::Vector3d t_enu0_body = gnss_tf::TfEnuEcef(t_ecef_body, gnss_tf::TfWgs84LlhEcef(t_ecef_enu0_));
        const Eigen::Quaterniond q_enu0_body = q_ecef_enu0_.inverse() * q_ecef_body;
        odom_enu0_msg.pose.pose.position = EigenToPointMsg(t_enu0_body);
        odom_enu0_msg.pose.pose.orientation = EigenToQuatMsg(q_enu0_body);
        // Cov
        Eigen::Matrix<double, 6, 6> cov_ecef(odom_msg.pose.covariance.data());
        const Eigen::Matrix3d rot_ecef_enu0 = q_ecef_enu0_.toRotationMatrix();
        Eigen::Map<Eigen::Matrix<double, 6, 6>> cov_enu0(odom_enu0_msg.pose.covariance.data());
        cov_enu0.topLeftCorner(3, 3) = rot_ecef_enu0 * cov_ecef.topLeftCorner(3, 3) * rot_ecef_enu0.transpose();
        cov_enu0.bottomRightCorner(3, 3) = rot_ecef_enu0 * cov_ecef.bottomRightCorner(3, 3) * rot_ecef_enu0.transpose();

        // Twist is the same as it is in the FP_POI frame
        odom_enu0_msg.twist = odom_msg.twist;

        // Publish
        odometry_enu0_pub_.publish(odom_enu0_msg);
    }
}
}  // namespace fixposition
