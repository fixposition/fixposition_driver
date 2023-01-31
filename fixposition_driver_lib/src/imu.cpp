/**
 *  @file
 *  @brief Implementation of ImuConverter converter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
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
    if (tokens.size() != 11) {
        std::cout << "Error in parsing IMU string with" << tokens.size() << "fields! IMU message will be empty.\n";
        msg_ = ImuData();
        return;
    }
    // header stamps
    msg_.stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));
    msg_.linear_acceleration = Vector3ToEigen(tokens.at(acc_x_idx), tokens.at(acc_y_idx), tokens.at(acc_z_idx));
    msg_.angular_velocity = Vector3ToEigen(tokens.at(rot_x_idx), tokens.at(rot_y_idx), tokens.at(rot_z_idx));

    // process all observers
    for (auto& ob : obs_) {
        ob(msg_);
    }
}

}  // namespace fixposition
