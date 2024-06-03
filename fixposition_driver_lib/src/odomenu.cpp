/**
 *  @file
 *  @brief Implementation of OdomenuConverter
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
#include <fixposition_driver_lib/converter/odomenu.hpp>
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

/**
 * @brief Parse status flag field
 *
 * @param[in] tokens list of tokens
 * @param[in] idx status flag index
 * @return int
 */
int ParseEnuStatusFlag(const std::vector<std::string>& tokens, const int idx) {
    if (tokens.at(idx).empty()) {
        return -1;
    } else {
        return std::stoi(tokens.at(idx));
    }
}

void OdomenuConverter::ConvertTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing Odomenu string with " << tokens.size()
                  << " fields! Odomenu message will be empty.\n";
    } else {
        // If size is ok, check version
        const int version = std::stoi(tokens.at(msg_version_idx));

        ok = version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing Odomenu string with version " << version
                      << " ! Odomenu message will be empty.\n";
        }
    }

    if (!ok) {
        // Reset message and return
        msgs_ = Msgs();
        return;
    }

    const int fusion_status = ParseEnuStatusFlag(tokens, fusion_status_idx);

    const bool fusion_init = fusion_status >= 3;

    // common data
    const auto stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));
    const Eigen::Vector3d t_enu0_enu =
        Vector3ToEigen(tokens.at(pos_x_idx), tokens.at(pos_y_idx), tokens.at(pos_z_idx));
    const Eigen::Quaterniond q_enu0_enu = Vector4ToEigen(tokens.at(orientation_w_idx), tokens.at(orientation_x_idx),
                                                         tokens.at(orientation_y_idx), tokens.at(orientation_z_idx));
    if (fusion_init) {
        // Populate odometry message header
        msgs_.odometry.stamp = stamp;
        msgs_.odometry.frame_id = "FP_ENU0";
        msgs_.odometry.child_frame_id = "FP_POI";

        // Pose & Cov
        msgs_.odometry.pose.position = (t_enu0_enu);
        msgs_.odometry.pose.orientation = (q_enu0_enu);
        msgs_.odometry.pose.cov = BuildCovMat6D(
            StringToDouble(tokens.at(pos_cov_xx_idx)), StringToDouble(tokens.at(pos_cov_yy_idx)),
            StringToDouble(tokens.at(pos_cov_zz_idx)), StringToDouble(tokens.at(pos_cov_xy_idx)),
            StringToDouble(tokens.at(pos_cov_yz_idx)), StringToDouble(tokens.at(pos_cov_xz_idx)),
            StringToDouble(tokens.at(orientation_cov_xx_idx)), StringToDouble(tokens.at(orientation_cov_yy_idx)),
            StringToDouble(tokens.at(orientation_cov_zz_idx)), StringToDouble(tokens.at(orientation_cov_xy_idx)),
            StringToDouble(tokens.at(orientation_cov_yz_idx)), StringToDouble(tokens.at(orientation_cov_xz_idx)));

        // Twist & Cov
        // Linear
        msgs_.odometry.twist.linear = Vector3ToEigen(tokens.at(vel_x_idx), tokens.at(vel_y_idx), tokens.at(vel_z_idx));
        // Angular
        msgs_.odometry.twist.angular = Vector3ToEigen(tokens.at(rot_x_idx), tokens.at(rot_y_idx), tokens.at(rot_z_idx));
        msgs_.odometry.twist.cov = BuildCovMat6D(
            StringToDouble(tokens.at(vel_cov_xx_idx)), StringToDouble(tokens.at(vel_cov_yy_idx)),
            StringToDouble(tokens.at(vel_cov_zz_idx)), StringToDouble(tokens.at(vel_cov_xy_idx)),
            StringToDouble(tokens.at(vel_cov_yz_idx)), StringToDouble(tokens.at(vel_cov_xz_idx)), 0, 0, 0, 0, 0, 0);

        // Euler angle wrt. ENU frame in the order of Yaw Pitch Roll
        msgs_.eul = gnss_tf::RotToEul(q_enu0_enu.toRotationMatrix());
    }

    // process all observers
    for (auto& ob : obs_) {
        ob(msgs_);
    }
}
}  // namespace fixposition
