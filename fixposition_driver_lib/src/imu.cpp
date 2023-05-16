/**
 *  @file
 *  @brief Implementation of ImuConverter converter
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

/* SYSTEM / STL */
#include <iostream>

/* PACKAGE */
#include <fixposition_driver_lib/converter/imu.hpp>

namespace fixposition {

/// msg field indices
static constexpr const int msg_type_idx = 1;
static constexpr const int msg_version_idx = 2;
static constexpr const int gps_week_idx = 3;
static constexpr const int gps_tow_idx = 4;
static constexpr const int acc_x_idx = 5;
static constexpr const int acc_y_idx = 6;
static constexpr const int acc_z_idx = 7;
static constexpr const int rot_x_idx = 8;
static constexpr const int rot_y_idx = 9;
static constexpr const int rot_z_idx = 10;

void ImuConverter::ConvertTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing IMU string with " << tokens.size() << " fields! IMU message will be empty.\n";

    } else {
        // If size is ok, check version
        const int version = std::stoi(tokens.at(msg_version_idx));

        ok = version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing IMU string with verion " << version << " ! IMU message will be empty.\n";
        }
    }

    if (!ok) {
        // Reset message and return
        msg_ = ImuData();
        return;
    }
    // header stamps
    msg_.stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));
    msg_.linear_acceleration = Vector3ToEigen(tokens.at(acc_x_idx), tokens.at(acc_y_idx), tokens.at(acc_z_idx));
    msg_.angular_velocity = Vector3ToEigen(tokens.at(rot_x_idx), tokens.at(rot_y_idx), tokens.at(rot_z_idx));
    msg_.frame_id = "FP_VRTK";
    
    // process all observers
    for (auto& ob : obs_) {
        ob(msg_);
    }
}

}  // namespace fixposition
