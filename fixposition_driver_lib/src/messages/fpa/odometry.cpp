/**
 *  @file
 *  @brief Implementation of FP_A-ODOMETRY parser
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

/* EXTERNAL */
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/* FIXPOSITION */
#include <fixposition_gnss_tf/gnss_tf.hpp>

/* PACKAGE */
#include <fixposition_driver_lib/messages/fpa_type.hpp>
#include <fixposition_driver_lib/messages/base_converter.hpp>
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
static constexpr const int gnss1_fix_type_idx = 23;
static constexpr const int gnss2_fix_type_idx = 24;
static constexpr const int wheelspeed_status_idx = 25;
static constexpr const int pos_cov_xx_idx = 26;
static constexpr const int pos_cov_yy_idx = 27;
static constexpr const int pos_cov_zz_idx = 28;
static constexpr const int pos_cov_xy_idx = 29;
static constexpr const int pos_cov_yz_idx = 30;
static constexpr const int pos_cov_xz_idx = 31;
static constexpr const int orientation_cov_xx_idx = 32;
static constexpr const int orientation_cov_yy_idx = 33;
static constexpr const int orientation_cov_zz_idx = 34;
static constexpr const int orientation_cov_xy_idx = 35;
static constexpr const int orientation_cov_yz_idx = 36;
static constexpr const int orientation_cov_xz_idx = 37;
static constexpr const int vel_cov_xx_idx = 38;
static constexpr const int vel_cov_yy_idx = 39;
static constexpr const int vel_cov_zz_idx = 40;
static constexpr const int vel_cov_xy_idx = 41;
static constexpr const int vel_cov_yz_idx = 42;
static constexpr const int vel_cov_xz_idx = 43;
static constexpr const int sw_version_idx = 44;

void FP_ODOMETRY::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing Odometry string with " << tokens.size()
                  << " fields! Odometry and status messages will be empty.\n";
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing Odometry string with version " << _version
                      << " ! Odometry and status messages will be empty.\n";
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // Status, regardless of fusion_init
    fusion_status = ParseStatusFlag(tokens, fusion_status_idx);
    imu_bias_status = ParseStatusFlag(tokens, imu_bias_status_idx);
    gnss1_status = ParseStatusFlag(tokens, gnss1_fix_type_idx);
    gnss2_status = ParseStatusFlag(tokens, gnss2_fix_type_idx);
    wheelspeed_status = ParseStatusFlag(tokens, wheelspeed_status_idx);
    version = tokens.at(sw_version_idx).empty() ? "UNKNOWN" : tokens.at(sw_version_idx);

    if (fusion_status >= 3) {
        // Populate VRTK message header
        odom.stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));;

        // Pose & Cov
        odom.pose.position = Vector3ToEigen(tokens.at(pos_x_idx), tokens.at(pos_y_idx), tokens.at(pos_z_idx));
        odom.pose.orientation = Vector4ToEigen(tokens.at(orientation_w_idx), tokens.at(orientation_x_idx),
                                          tokens.at(orientation_y_idx), tokens.at(orientation_z_idx));
        odom.pose.cov = BuildCovMat6D(
            StringToDouble(tokens.at(pos_cov_xx_idx)), StringToDouble(tokens.at(pos_cov_yy_idx)),
            StringToDouble(tokens.at(pos_cov_zz_idx)), StringToDouble(tokens.at(pos_cov_xy_idx)),
            StringToDouble(tokens.at(pos_cov_yz_idx)), StringToDouble(tokens.at(pos_cov_xz_idx)),
            StringToDouble(tokens.at(orientation_cov_xx_idx)), StringToDouble(tokens.at(orientation_cov_yy_idx)),
            StringToDouble(tokens.at(orientation_cov_zz_idx)), StringToDouble(tokens.at(orientation_cov_xy_idx)),
            StringToDouble(tokens.at(orientation_cov_yz_idx)), StringToDouble(tokens.at(orientation_cov_xz_idx)));

        // Twist & Cov
        // Linear
        odom.twist.linear = Vector3ToEigen(tokens.at(vel_x_idx), tokens.at(vel_y_idx), tokens.at(vel_z_idx));
        // Angular
        odom.twist.angular = Vector3ToEigen(tokens.at(rot_x_idx), tokens.at(rot_y_idx), tokens.at(rot_z_idx));
        odom.twist.cov = BuildCovMat6D(
            StringToDouble(tokens.at(vel_cov_xx_idx)), StringToDouble(tokens.at(vel_cov_yy_idx)),
            StringToDouble(tokens.at(vel_cov_zz_idx)), StringToDouble(tokens.at(vel_cov_xy_idx)),
            StringToDouble(tokens.at(vel_cov_yz_idx)), StringToDouble(tokens.at(vel_cov_xz_idx)), 0, 0, 0, 0, 0, 0);

        acceleration = Vector3ToEigen(tokens.at(acc_x_idx), tokens.at(acc_y_idx), tokens.at(acc_z_idx));
    }
}

}  // namespace fixposition
