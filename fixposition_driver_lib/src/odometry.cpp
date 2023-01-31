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

/* FIXPOSITION */
#include <fixposition_gnss_tf/gnss_tf.hpp>

/* PACKAGE */
#include <fixposition_driver_lib/converter/odometry.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

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
int ParseStatusFlag(const std::vector<std::string>& tokens, const int idx) {
    if (tokens.at(idx).empty()) {
        return -1;
    } else {
        return std::stoi(tokens.at(idx));
    }
}

void OdometryConverter::ConvertTokens(const std::vector<std::string>& tokens) {
    if (tokens.size() != 44) {
        std::cout << "Error in parsing Odometry string with " << tokens.size()
                  << " fields! odometry and status messages will be empty.\n";

        msgs_ = Msgs();
        return;
    }
    const int fusion_status = ParseStatusFlag(tokens, fusion_status_idx);

    const bool fusion_init = fusion_status >= 3;

    // common data
    const auto stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));
    const Eigen::Vector3d t_ecef_body =
        Vector3ToEigen(tokens.at(pos_x_idx), tokens.at(pos_y_idx), tokens.at(pos_z_idx));
    const Eigen::Quaterniond q_ecef_body = Vector4ToEigen(tokens.at(orientation_w_idx), tokens.at(orientation_x_idx),
                                                          tokens.at(orientation_y_idx), tokens.at(orientation_z_idx));
    if (fusion_init) {
        //  TFs
        msgs_.tf_ecef_enu.stamp = stamp;
        msgs_.tf_ecef_poi.stamp = stamp;
        msgs_.tf_ecef_enu.frame_id = "ECEF";
        msgs_.tf_ecef_poi.frame_id = "ECEF";
        msgs_.tf_ecef_poi.child_frame_id = "FP_POI";
        msgs_.tf_ecef_enu.child_frame_id = "FP_ENU";  // The ENU frame at the position of FP_POI

        // static TF ECEF ENU0
        if (!tf_ecef_enu0_set_ && msgs_.tf_ecef_enu0.translation.isZero()) {
            // ENU0 frame is not yet set, set the same ENU tf to ENU0
            t_ecef_enu0_ = t_ecef_body;
            q_ecef_enu0_ = Eigen::Quaterniond(gnss_tf::RotEnuEcef(t_ecef_body).transpose());
            msgs_.tf_ecef_enu0.translation = t_ecef_enu0_;
            msgs_.tf_ecef_enu0.rotation = q_ecef_enu0_;
            tf_ecef_enu0_set_ = true;
        }

        // TF ECEF POI is basically the same as the odometry, containing the Pose of the POI in the ECEF Frame
        msgs_.tf_ecef_poi.translation = (t_ecef_body);
        msgs_.tf_ecef_poi.rotation = (q_ecef_body);

        // Quaternion from ECEF to Local ENU at POI
        const Eigen::Quaterniond q_ecef_enu = Eigen::Quaterniond(gnss_tf::RotEnuEcef(t_ecef_body).transpose());
        // TF ECEF ENU
        msgs_.tf_ecef_enu.translation = (t_ecef_body);
        msgs_.tf_ecef_enu.rotation = (q_ecef_enu);

        // Send TFs
        if (CheckQuat(msgs_.tf_ecef_poi.rotation)) {
        }
        if (CheckQuat(msgs_.tf_ecef_enu.rotation)) {
        }
        // Send Static TF ECEF ENU0
        if (tf_ecef_enu0_set_ && CheckQuat(msgs_.tf_ecef_enu0.rotation)) {
        }

        msgs_.odometry.stamp = stamp;
        msgs_.odometry.frame_id = "ECEF";
        msgs_.odometry.child_frame_id = "FP_POI";

        msgs_.vrtk.stamp = stamp;
        msgs_.vrtk.frame_id = "ECEF";
        msgs_.vrtk.pose_frame = "FP_POI";
        msgs_.vrtk.kin_frame = "FP_POI";

        // Pose & Cov
        msgs_.odometry.pose.position = (t_ecef_body);
        msgs_.odometry.pose.orientation = (q_ecef_body);
        msgs_.odometry.pose.cov = BuildCovMat6D(
            StringToDouble(tokens.at(pos_cov_xx_idx)), StringToDouble(tokens.at(pos_cov_yy_idx)),
            StringToDouble(tokens.at(pos_cov_zz_idx)), StringToDouble(tokens.at(pos_cov_xy_idx)),
            StringToDouble(tokens.at(pos_cov_yz_idx)), StringToDouble(tokens.at(pos_cov_xz_idx)),
            StringToDouble(tokens.at(orientation_cov_xx_idx)), StringToDouble(tokens.at(orientation_cov_yy_idx)),
            StringToDouble(tokens.at(orientation_cov_zz_idx)), StringToDouble(tokens.at(orientation_cov_xy_idx)),
            StringToDouble(tokens.at(orientation_cov_yz_idx)), StringToDouble(tokens.at(orientation_cov_xz_idx)));
        msgs_.vrtk.pose = msgs_.odometry.pose;

        // Twist & Cov
        // Linear
        msgs_.odometry.twist.linear = Vector3ToEigen(tokens.at(vel_x_idx), tokens.at(vel_y_idx), tokens.at(vel_z_idx));
        // Angular
        msgs_.odometry.twist.angular = Vector3ToEigen(tokens.at(rot_x_idx), tokens.at(rot_y_idx), tokens.at(rot_z_idx));
        msgs_.odometry.twist.cov = BuildCovMat6D(
            StringToDouble(tokens.at(vel_cov_xx_idx)), StringToDouble(tokens.at(vel_cov_yy_idx)),
            StringToDouble(tokens.at(vel_cov_zz_idx)), StringToDouble(tokens.at(vel_cov_xy_idx)),
            StringToDouble(tokens.at(vel_cov_yz_idx)), StringToDouble(tokens.at(vel_cov_xz_idx)), 0, 0, 0, 0, 0, 0);
        msgs_.vrtk.velocity = msgs_.odometry.twist;

        // Euler angle wrt. ENU frame in the order of Yaw Pitch Roll
        msgs_.eul = gnss_tf::EcefPoseToEnuEul(t_ecef_body, q_ecef_body.toRotationMatrix());

        // Odmetry msg ENU0 - FP_POI
        msgs_.odometry_enu0.stamp = stamp;
        msgs_.odometry_enu0.frame_id = "ENU0";
        msgs_.odometry_enu0.child_frame_id = "FP_POI";
        // Pose
        // convert position in ECEF into position in ENU0
        const Eigen::Vector3d t_enu0_body = gnss_tf::TfEnuEcef(t_ecef_body, gnss_tf::TfWgs84LlhEcef(t_ecef_enu0_));
        const Eigen::Quaterniond q_enu0_body = q_ecef_enu0_.inverse() * q_ecef_body;
        msgs_.odometry_enu0.pose.position = (t_enu0_body);
        msgs_.odometry_enu0.pose.orientation = (q_enu0_body);
        // Cov
        Eigen::Matrix<double, 6, 6> cov_ecef(msgs_.odometry.pose.cov);
        const Eigen::Matrix3d rot_ecef_enu0 = q_ecef_enu0_.toRotationMatrix();
        Eigen::Map<Eigen::Matrix<double, 6, 6>> cov_enu0(msgs_.odometry_enu0.pose.cov.data());
        msgs_.odometry_enu0.pose.cov.topLeftCorner(3, 3) =
            rot_ecef_enu0 * cov_ecef.topLeftCorner(3, 3) * rot_ecef_enu0.transpose();
        msgs_.odometry_enu0.pose.cov.bottomRightCorner(3, 3) =
            rot_ecef_enu0 * cov_ecef.bottomRightCorner(3, 3) * rot_ecef_enu0.transpose();

        // Twist is the same as it is in the FP_POI frame
        msgs_.odometry_enu0.twist = msgs_.odometry.twist;
    }

    // Msgs
    //!<  Odmetry msg ECEF - FP_POI

    // Status, regardless of fusion_init
    msgs_.vrtk.fusion_status = fusion_status;
    msgs_.vrtk.imu_bias_status = ParseStatusFlag(tokens, imu_bias_status_idx);
    msgs_.vrtk.gnss_status = ParseStatusFlag(tokens, gnss_fix_type_idx);
    msgs_.vrtk.wheelspeed_status = ParseStatusFlag(tokens, wheelspeed_status_idx);
    msgs_.vrtk.version = tokens.at(sw_version_idx).empty() ? "UNKNOWN" : tokens.at(sw_version_idx);

    // POI IMU Message
    msgs_.imu.stamp = stamp;
    msgs_.imu.frame_id = "FP_POI";
    // Omega
    msgs_.imu.angular_velocity = msgs_.odometry.twist.angular;
    // Acceleration
    msgs_.imu.linear_acceleration = Vector3ToEigen(tokens.at(acc_x_idx), tokens.at(acc_y_idx), tokens.at(acc_z_idx));

    // process all observers
    for (auto& ob : obs_) {
        ob(msgs_);
    }
}
}  // namespace fixposition
