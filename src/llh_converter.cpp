/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /
 *   /  /\  \
 *  /__/  \__\  Fixposition AG
 *
 * @file fp_msg_converter.cpp
 * @author Kailin Huang (kailin.huang@fixposition.com)
 * @brief
 * @date 2022-01-26
 *
 */
#include <fixposition_driver/converter/llh_converter.hpp>
#include <fixposition_driver/time_conversions.hpp>

namespace fixposition {

void LlhConverter::ConvertTokensAndPublish(const std::vector<std::string>& tokens) {
    sensor_msgs::NavSatFix navsat_msg;
    if (tokens.size() != 12) {
        ROS_DEBUG_STREAM("Error in parsing LLH string! NavSatFix message will be empty.");
        return;
    }
    if (!tokens.at(1).empty() && !tokens.at(2).empty()) {
        times::GpsTime gps_time(std::stoi(tokens.at(1)), std::stod(tokens.at(2)));
        navsat_msg.header.stamp = times::GpsTimeToRosTime(gps_time);

    } else {
        ROS_DEBUG_STREAM("GPS time empty. Replacing with current ROS time.");
        navsat_msg.header.stamp = ros::Time::now();
    }
    navsat_msg.header.frame_id = "VRTK Output";
    navsat_msg.latitude = tokens.at(3).empty() ? 0 : std::stod(tokens.at(3));
    navsat_msg.longitude = tokens.at(4).empty() ? 0 : std::stod(tokens.at(4));
    navsat_msg.altitude = tokens.at(5).empty() ? 0 : std::stod(tokens.at(5));

    // Covariance diagonals
    navsat_msg.position_covariance[0] = tokens.at(6).empty() ? 0 : std::stod(tokens.at(6));
    navsat_msg.position_covariance[4] = tokens.at(7).empty() ? 0 : std::stod(tokens.at(7));
    navsat_msg.position_covariance[8] = tokens.at(8).empty() ? 0 : std::stod(tokens.at(8));

    // Rest of covariance fields
    navsat_msg.position_covariance[1] = navsat_msg.position_covariance[3] =
        tokens.at(9).empty() ? 0 : std::stod(tokens.at(9));
    navsat_msg.position_covariance[2] = navsat_msg.position_covariance[6] =
        tokens.at(11).empty() ? 0 : std::stod(tokens.at(11));
    navsat_msg.position_covariance[5] = navsat_msg.position_covariance[7] =
        tokens.at(10).empty() ? 0 : std::stod(tokens.at(10));
    navsat_msg.position_covariance_type = 3;

    navsatfix_pub_.publish(navsat_msg);
}

}  // namespace fixposition