/**
 *  @file
 *  @brief Implementation of FP_A-ODOMENU parser
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

/* PACKAGE */
#include <fixposition_driver_lib/messages/fpa_type.hpp>
#include <fixposition_driver_lib/messages/base_converter.hpp>

namespace fixposition {

/// msg field indices
static constexpr int msg_type_idx = 1;
static constexpr int msg_version_idx = 2;
static constexpr int gps_week_idx = 3;
static constexpr int gps_tow_idx = 4;
static constexpr int pos_x_idx = 5;
static constexpr int pos_y_idx = 6;
static constexpr int pos_z_idx = 7;
static constexpr int orientation_w_idx = 8;
static constexpr int orientation_x_idx = 9;
static constexpr int orientation_y_idx = 10;
static constexpr int orientation_z_idx = 11;
static constexpr int vel_x_idx = 12;
static constexpr int vel_y_idx = 13;
static constexpr int vel_z_idx = 14;
static constexpr int rot_x_idx = 15;
static constexpr int rot_y_idx = 16;
static constexpr int rot_z_idx = 17;
static constexpr int acc_x_idx = 18;
static constexpr int acc_y_idx = 19;
static constexpr int acc_z_idx = 20;
static constexpr int fusion_status_idx = 21;
static constexpr int imu_bias_status_idx = 22;
static constexpr int gnss1_fix_type_idx = 23;
static constexpr int gnss2_fix_type_idx = 24;
static constexpr int wheelspeed_status_idx = 25;
static constexpr int pos_cov_xx_idx = 26;
static constexpr int pos_cov_yy_idx = 27;
static constexpr int pos_cov_zz_idx = 28;
static constexpr int pos_cov_xy_idx = 29;
static constexpr int pos_cov_yz_idx = 30;
static constexpr int pos_cov_xz_idx = 31;
static constexpr int orientation_cov_xx_idx = 32;
static constexpr int orientation_cov_yy_idx = 33;
static constexpr int orientation_cov_zz_idx = 34;
static constexpr int orientation_cov_xy_idx = 35;
static constexpr int orientation_cov_yz_idx = 36;
static constexpr int orientation_cov_xz_idx = 37;
static constexpr int vel_cov_xx_idx = 38;
static constexpr int vel_cov_yy_idx = 39;
static constexpr int vel_cov_zz_idx = 40;
static constexpr int vel_cov_xy_idx = 41;
static constexpr int vel_cov_yz_idx = 42;
static constexpr int vel_cov_xz_idx = 43;

void FP_ODOMENU::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing FP_A-ODOMENU string with " << tokens.size() << " fields!\n";
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing FP_A-ODOMENU string with version " << _version << "!\n";
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // VRTK status flags
    fusion_status = ParseStatusFlag(tokens, fusion_status_idx);
    imu_bias_status = ParseStatusFlag(tokens, imu_bias_status_idx);
    gnss1_status = ParseStatusFlag(tokens, gnss1_fix_type_idx);
    gnss2_status = ParseStatusFlag(tokens, gnss2_fix_type_idx);
    wheelspeed_status = ParseStatusFlag(tokens, wheelspeed_status_idx);

    if (fusion_status >= static_cast<int>(FusionStatus::INERTIAL_GNSS_FUSION)) {
        // Message header
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
        odom.twist.linear = Vector3ToEigen(tokens.at(vel_x_idx), tokens.at(vel_y_idx), tokens.at(vel_z_idx));
        odom.twist.angular = Vector3ToEigen(tokens.at(rot_x_idx), tokens.at(rot_y_idx), tokens.at(rot_z_idx));
        odom.twist.cov = BuildCovMat6D(
            StringToDouble(tokens.at(vel_cov_xx_idx)), StringToDouble(tokens.at(vel_cov_yy_idx)),
            StringToDouble(tokens.at(vel_cov_zz_idx)), StringToDouble(tokens.at(vel_cov_xy_idx)),
            StringToDouble(tokens.at(vel_cov_yz_idx)), StringToDouble(tokens.at(vel_cov_xz_idx)), 0, 0, 0, 0, 0, 0);

        // Acceleration
        acceleration = Vector3ToEigen(tokens.at(acc_x_idx), tokens.at(acc_y_idx), tokens.at(acc_z_idx));
    }
}

}  // namespace fixposition
