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

/* PACKAGE */
#include <fixposition_driver/converter/imu.hpp>

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

void ImuConverter::ConvertTokensAndPublish(const std::vector<std::string>& tokens) {
    sensor_msgs::Imu msg;
    if (tokens.size() != 11) {
        ROS_INFO("Error in parsing IMU string with %lu fields! IMU message will be empty.", tokens.size());
        return;
    }
    // header stamps
    msg.header.stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));
    msg.linear_acceleration.x = StringToDouble(tokens.at(acc_x_idx));
    msg.linear_acceleration.y = StringToDouble(tokens.at(acc_y_idx));
    msg.linear_acceleration.z = StringToDouble(tokens.at(acc_z_idx));

    msg.angular_velocity.x = StringToDouble(tokens.at(rot_x_idx));
    msg.angular_velocity.y = StringToDouble(tokens.at(rot_y_idx));
    msg.angular_velocity.z = StringToDouble(tokens.at(rot_z_idx));
    if (imu_pub_.getNumSubscribers() > 0) {
        imu_pub_.publish(msg);
    }
}

}  // namespace fixposition