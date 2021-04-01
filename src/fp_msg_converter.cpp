/**
 * @file fp_msg_converter.cpp
 * @author Andreea Lutac (andreea.lutac@fixposition.ch)
 * @date 2020-04-06
 * @version 0.1
 *
 * @copyright @ Fixposition AG (c) 2017 - 2020
 *
 * @brief
 *
 * @details
 */

#include "fp_msg_converter.hpp"

FpMsgConverter::FpMsgConverter() : BaseConverter() {}
void FpMsgConverter::convertAndPublish(const std::string& state, const ros::Publisher& odometry_pub,
                                       const ros::Publisher& imu_pub, const ros::Publisher& navsat_pub,
                                       const ros::Publisher& status_pub) {
    if (!state.empty()) {
        std::size_t star = state.find_last_of("*");
        std::string state_data = state.substr(0, star);
        state_data = state_data.erase(0, 1);

        unsigned int checksum = std::stoul(state.substr(star + 1, 2), nullptr, 16);
        bool ret = verify_checksum(state_data, checksum);
        if (!ret) {
            ROS_DEBUG_STREAM("Checksum invalid! Odometry message will be empty.");
            return;
        } else {
            ROS_DEBUG_STREAM("Checksum valid.");
        }

        std::vector<std::string> tokens;
        split_message(state_data, ",", &tokens);
        for (std::string token : tokens) {
            ROS_DEBUG_STREAM("Token: " << token);
        }

        if (tokens[0] == "FP") {
            ROS_DEBUG_STREAM("Received FP message.");
            handleFPMessage(tokens, odometry_pub, imu_pub, status_pub);
        } else if (tokens[0] == "LLH") {
            ROS_DEBUG_STREAM("Received LLH message.");
            handleLLHMessage(tokens, navsat_pub);
        } else {
            ROS_DEBUG_STREAM("Unknown message type.");
            return;
        }

    } else {
        ROS_ERROR_STREAM("State message empty! Not well parsed?");
    }
}

void FpMsgConverter::handleFPMessage(const std::vector<std::string>& tokens, const ros::Publisher& odometry_pub,
                                     const ros::Publisher& imu_pub, const ros::Publisher& status_pub) {
    nav_msgs::Odometry odom_msg;
    sensor_msgs::Imu imu_msg;
    fixposition_output::VRTK status_msg;
    if (tokens.size() != 41) {
        ROS_DEBUG_STREAM("Error in parsing FP string! Odometry, status and IMU messages will be empty.");
        return;
    }
    if (!tokens.at(1).empty() && !tokens.at(2).empty()) {
        GPSWeekSec weeksec(std::stoi(tokens.at(1)), std::stod(tokens.at(2)));
        odom_msg.header.stamp = GPSWeekSec2RosTime(weeksec);
        imu_msg.header.stamp = GPSWeekSec2RosTime(weeksec);
        status_msg.header.stamp = GPSWeekSec2RosTime(weeksec);

    } else {
        ROS_DEBUG_STREAM("GPS time empty. Replacing with current ROS time.");
        odom_msg.header.stamp = ros::Time::now();
        imu_msg.header.stamp = ros::Time::now();
        status_msg.header.stamp = ros::Time::now();
    }
    odom_msg.header.frame_id = status_msg.header.frame_id = "ECEF";
    odom_msg.child_frame_id = status_msg.pose_frame = "ECEF";
    imu_msg.header.frame_id = status_msg.kin_frame = "POI";

    odom_msg.pose.pose.position.x = status_msg.pose.pose.position.x =
        tokens.at(3).empty() ? 0 : std::stod(tokens.at(3));
    odom_msg.pose.pose.position.y = status_msg.pose.pose.position.y =
        tokens.at(4).empty() ? 0 : std::stod(tokens.at(4));
    odom_msg.pose.pose.position.z = status_msg.pose.pose.position.z =
        tokens.at(5).empty() ? 0 : std::stod(tokens.at(5));

    odom_msg.pose.pose.orientation.w = status_msg.pose.pose.orientation.w =
        tokens.at(6).empty() ? 1 : std::stod(tokens.at(6));
    odom_msg.pose.pose.orientation.x = status_msg.pose.pose.orientation.x =
        tokens.at(7).empty() ? 0 : std::stod(tokens.at(7));
    odom_msg.pose.pose.orientation.y = status_msg.pose.pose.orientation.y =
        tokens.at(8).empty() ? 0 : std::stod(tokens.at(8));
    odom_msg.pose.pose.orientation.z = status_msg.pose.pose.orientation.z =
        tokens.at(9).empty() ? 0 : std::stod(tokens.at(9));

    odom_msg.twist.twist.linear.x = status_msg.velocity.twist.linear.x =
        tokens.at(10).empty() ? 0 : std::stod(tokens.at(10));
    odom_msg.twist.twist.linear.y = status_msg.velocity.twist.linear.y =
        tokens.at(11).empty() ? 0 : std::stod(tokens.at(11));
    odom_msg.twist.twist.linear.z = status_msg.velocity.twist.linear.z =
        tokens.at(12).empty() ? 0 : std::stod(tokens.at(12));

    imu_msg.angular_velocity.x = odom_msg.twist.twist.angular.x = status_msg.velocity.twist.angular.x =
        tokens.at(13).empty() ? 0 : std::stod(tokens.at(13));
    imu_msg.angular_velocity.y = odom_msg.twist.twist.angular.y = status_msg.velocity.twist.angular.y =
        tokens.at(14).empty() ? 0 : std::stod(tokens.at(14));
    imu_msg.angular_velocity.z = odom_msg.twist.twist.angular.z = status_msg.velocity.twist.angular.z =
        tokens.at(15).empty() ? 0 : std::stod(tokens.at(15));

    imu_msg.linear_acceleration.x = tokens.at(16).empty() ? 0 : std::stod(tokens.at(16));
    imu_msg.linear_acceleration.y = tokens.at(17).empty() ? 0 : std::stod(tokens.at(17));
    imu_msg.linear_acceleration.z = tokens.at(18).empty() ? 0 : std::stod(tokens.at(18));

    status_msg.fusion_status = tokens.at(19).empty() ? tokens.at(19)[0] - '0' : 0;
    status_msg.imu_bias_status = tokens.at(20).empty() ? tokens.at(20)[0] - '0' : 0;
    status_msg.gnss_status = tokens.at(21).empty() ? tokens.at(21)[0] - '0' : 0;

    std::vector<double> pose_cov = buildCovMatrix12(
        tokens.at(22).empty() ? 0 : std::stod(tokens.at(22)), tokens.at(23).empty() ? 0 : std::stod(tokens.at(23)),
        tokens.at(24).empty() ? 0 : std::stod(tokens.at(24)), tokens.at(25).empty() ? 0 : std::stod(tokens.at(25)),
        tokens.at(26).empty() ? 0 : std::stod(tokens.at(26)), tokens.at(27).empty() ? 0 : std::stod(tokens.at(27)),
        tokens.at(28).empty() ? 0 : std::stod(tokens.at(28)), tokens.at(29).empty() ? 0 : std::stod(tokens.at(29)),
        tokens.at(30).empty() ? 0 : std::stod(tokens.at(30)), tokens.at(31).empty() ? 0 : std::stod(tokens.at(31)),
        tokens.at(32).empty() ? 0 : std::stod(tokens.at(32)), tokens.at(33).empty() ? 0 : std::stod(tokens.at(33)));
    std::vector<double> vel_cov = buildCovMatrix12(
        tokens.at(34).empty() ? 0 : std::stod(tokens.at(34)), tokens.at(35).empty() ? 0 : std::stod(tokens.at(35)),
        tokens.at(36).empty() ? 0 : std::stod(tokens.at(36)), tokens.at(37).empty() ? 0 : std::stod(tokens.at(37)),
        tokens.at(38).empty() ? 0 : std::stod(tokens.at(38)), tokens.at(39).empty() ? 0 : std::stod(tokens.at(39)), 0,
        0, 0, 0, 0, 0);

    std::copy(pose_cov.begin(), pose_cov.end(), odom_msg.pose.covariance.begin());
    std::copy(pose_cov.begin(), pose_cov.end(), status_msg.pose.covariance.begin());
    std::copy(vel_cov.begin(), vel_cov.end(), odom_msg.twist.covariance.begin());
    std::copy(vel_cov.begin(), vel_cov.end(), status_msg.velocity.covariance.begin());

    status_msg.version = tokens.at(40).empty() ? "" : tokens.at(40);

    odometry_pub.publish(odom_msg);
    imu_pub.publish(imu_msg);
    status_pub.publish(status_msg);
}

void FpMsgConverter::handleLLHMessage(const std::vector<std::string>& tokens, const ros::Publisher& navsat_pub) {
    sensor_msgs::NavSatFix navsat_msg;
    if (tokens.size() != 12) {
        ROS_DEBUG_STREAM("Error in parsing LLH string! NavSatFix message will be empty.");
        return;
    }
    if (!tokens.at(1).empty() && !tokens.at(2).empty()) {
        GPSWeekSec weeksec(std::stoi(tokens.at(1)), std::stod(tokens.at(2)));
        navsat_msg.header.stamp = GPSWeekSec2RosTime(weeksec);

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

    navsat_pub.publish(navsat_msg);
}

std::vector<double> FpMsgConverter::buildCovMatrix12(double xx, double yy, double zz, double xy, double yz, double xz,
                                                     double xx1, double yy1, double zz1, double xy1, double yz1,
                                                     double xz1) {
    std::vector<double> cov_mat(36, 0);

    // Diagonals
    cov_mat[0 + 6 * 0] = xx;   // 0
    cov_mat[1 + 6 * 1] = yy;   // 7
    cov_mat[2 + 6 * 2] = zz;   // 14
    cov_mat[3 + 6 * 3] = xx1;  // 21
    cov_mat[4 + 6 * 4] = yy1;  // 28
    cov_mat[5 + 6 * 5] = zz1;  // 35

    // Rest of values
    cov_mat[1 + 6 * 0] = cov_mat[0 + 6 * 1] = xy;   // 1 = 6
    cov_mat[2 + 6 * 1] = cov_mat[1 + 6 * 2] = yz;   // 8 = 13
    cov_mat[2 + 6 * 0] = cov_mat[0 + 6 * 2] = xz;   // 2 = 12
    cov_mat[4 + 6 * 3] = cov_mat[3 + 6 * 4] = xy1;  // 22 = 27
    cov_mat[5 + 6 * 4] = cov_mat[4 + 6 * 5] = yz1;  // 29 = 34
    cov_mat[5 + 6 * 3] = cov_mat[3 + 6 * 5] = xz1;  // 23 = 33
    return cov_mat;
}