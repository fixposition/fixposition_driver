/**
 *  @file
 *  @brief Implementation of FP_A-IMUBIAS parser
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
static constexpr int fusion_imu_idx = 5;
static constexpr int imu_status_idx = 6;
static constexpr int imu_noise_idx = 7;
static constexpr int imu_conv_idx = 8;
static constexpr int bias_acc_x_idx = 9;
static constexpr int bias_acc_y_idx = 10;
static constexpr int bias_acc_z_idx = 11;
static constexpr int bias_gyr_x_idx = 12;
static constexpr int bias_gyr_y_idx = 13;
static constexpr int bias_gyr_z_idx = 14;
static constexpr int bias_cov_acc_x_idx = 15;
static constexpr int bias_cov_acc_y_idx = 16;
static constexpr int bias_cov_acc_z_idx = 17;
static constexpr int bias_cov_gyr_x_idx = 18;
static constexpr int bias_cov_gyr_y_idx = 19;
static constexpr int bias_cov_gyr_z_idx = 20;

void FP_IMUBIAS::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing FP_A-IMUBIAS string with " << tokens.size() << " fields!\n";
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing FP_A-IMUBIAS string with version " << _version << "!\n";
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // Parse time
    stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));

    // Populate fields
    frame_id = "FP_VRTK";
    fusion_imu = ParseStatusFlag(tokens, fusion_imu_idx);
    imu_status = ParseStatusFlag(tokens, imu_status_idx);
    imu_noise = ParseStatusFlag(tokens, imu_noise_idx);
    imu_conv = ParseStatusFlag(tokens, imu_conv_idx);
    bias_acc = Vector3ToEigen(tokens.at(bias_acc_x_idx), tokens.at(bias_acc_y_idx), tokens.at(bias_acc_z_idx));
    bias_gyr = Vector3ToEigen(tokens.at(bias_gyr_x_idx), tokens.at(bias_gyr_y_idx), tokens.at(bias_gyr_z_idx));
    bias_cov_acc = Vector3ToEigen(tokens.at(bias_cov_acc_x_idx), tokens.at(bias_cov_acc_y_idx), tokens.at(bias_cov_acc_z_idx));
    bias_cov_gyr = Vector3ToEigen(tokens.at(bias_cov_gyr_x_idx), tokens.at(bias_cov_gyr_y_idx), tokens.at(bias_cov_gyr_z_idx));
}

}  // namespace fixposition
