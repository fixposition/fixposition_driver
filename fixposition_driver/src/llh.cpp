/**
 *  @file
 *  @brief Implementation of LlhConverter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* PACKAGE */
#include <fixposition_driver/converter/llh.hpp>

namespace fixposition {

/// msg field indices
static constexpr const int msg_type_idx = 1;
static constexpr const int msg_versio_idx = 2;
static constexpr const int gps_week_idx = 3;
static constexpr const int gps_tow_idx = 4;
static constexpr const int latitude_idx = 5;
static constexpr const int longitude_idx = 6;
static constexpr const int height_idx = 7;
static constexpr const int pos_cov_ee_idx = 8;
static constexpr const int pos_cov_nn_idx = 9;
static constexpr const int pos_cov_uu_idx = 10;
static constexpr const int pos_cov_en_idx = 11;
static constexpr const int pos_cov_nu_idx = 12;
static constexpr const int pos_cov_eu_idx = 13;

void LlhConverter::ConvertTokensAndPublish(const std::vector<std::string>& tokens) {
    sensor_msgs::NavSatFix navsat_msg;
    if (tokens.size() != 14) {
        ROS_INFO("Error in parsing LLH string with %lu fields! NavSatFix message will be empty.", tokens.size());
        return;
    }
    // header stamps
    navsat_msg.header.stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));

    navsat_msg.header.frame_id = "POI";
    navsat_msg.latitude = StringToDouble(tokens.at(latitude_idx));
    navsat_msg.longitude = StringToDouble(tokens.at(longitude_idx));
    navsat_msg.altitude = StringToDouble(tokens.at(height_idx));

    // Covariance diagonals
    navsat_msg.position_covariance[0] = StringToDouble(tokens.at(pos_cov_ee_idx));
    navsat_msg.position_covariance[4] = StringToDouble(tokens.at(pos_cov_nn_idx));
    navsat_msg.position_covariance[8] = StringToDouble(tokens.at(pos_cov_uu_idx));

    // Rest of covariance fields
    navsat_msg.position_covariance[1] = navsat_msg.position_covariance[3] = StringToDouble(tokens.at(pos_cov_en_idx));
    navsat_msg.position_covariance[2] = navsat_msg.position_covariance[6] = StringToDouble(tokens.at(pos_cov_nu_idx));
    navsat_msg.position_covariance[5] = navsat_msg.position_covariance[7] = StringToDouble(tokens.at(pos_cov_eu_idx));
    navsat_msg.position_covariance_type = 3;

    navsatfix_pub_.publish(navsat_msg);
}

}  // namespace fixposition
