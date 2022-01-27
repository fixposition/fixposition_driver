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

/* SYSTEM / STL */

/* EXTERNAL */

/* ROS */

/* PACKAGE */
#include <fixposition_driver/converter/fp_converter.hpp>
#include <fixposition_driver/time_conversions.hpp>

namespace fixposition {

void FpConverter::ConvertTokensAndPublish(const std::vector<std::string>& tokens) {
    nav_msgs::Odometry odom_msg;
    sensor_msgs::Imu imu_msg;
    fixposition_driver::VRTK status_msg;
    if (tokens.size() != 41) {
        ROS_DEBUG_STREAM("Error in parsing FP string! Odometry, status and IMU messages will be empty.");
        return;
    }
    if (!tokens.at(1).empty() && !tokens.at(2).empty()) {
        times::GpsTime gps_time(std::stoi(tokens.at(1)), std::stod(tokens.at(2)));
        odom_msg.header.stamp = times::GpsTimeToRosTime(gps_time);
        imu_msg.header.stamp = times::GpsTimeToRosTime(gps_time);
        status_msg.header.stamp = times::GpsTimeToRosTime(gps_time);

    } else {
        ROS_DEBUG_STREAM("GPS time empty. Replacing with current ROS time.");
        odom_msg.header.stamp = ros::Time::now();
        imu_msg.header.stamp = ros::Time::now();
        status_msg.header.stamp = ros::Time::now();
    }
    odom_msg.header.frame_id = status_msg.header.frame_id = "ECEF";
    odom_msg.child_frame_id = status_msg.pose_frame = "POI";
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

    status_msg.fusion_status = tokens.at(19).empty() ? 0 : tokens.at(19)[0] - '0';
    status_msg.imu_bias_status = tokens.at(20).empty() ? 0 : tokens.at(20)[0] - '0';
    status_msg.gnss_status = tokens.at(21).empty() ? 0 : tokens.at(21)[0] - '0';

    std::vector<double> pose_cov = BuildCovMat6D(
        tokens.at(22).empty() ? 0 : std::stod(tokens.at(22)), tokens.at(23).empty() ? 0 : std::stod(tokens.at(23)),
        tokens.at(24).empty() ? 0 : std::stod(tokens.at(24)), tokens.at(25).empty() ? 0 : std::stod(tokens.at(25)),
        tokens.at(26).empty() ? 0 : std::stod(tokens.at(26)), tokens.at(27).empty() ? 0 : std::stod(tokens.at(27)),
        tokens.at(28).empty() ? 0 : std::stod(tokens.at(28)), tokens.at(29).empty() ? 0 : std::stod(tokens.at(29)),
        tokens.at(30).empty() ? 0 : std::stod(tokens.at(30)), tokens.at(31).empty() ? 0 : std::stod(tokens.at(31)),
        tokens.at(32).empty() ? 0 : std::stod(tokens.at(32)), tokens.at(33).empty() ? 0 : std::stod(tokens.at(33)));
    std::vector<double> vel_cov = BuildCovMat6D(
        tokens.at(34).empty() ? 0 : std::stod(tokens.at(34)), tokens.at(35).empty() ? 0 : std::stod(tokens.at(35)),
        tokens.at(36).empty() ? 0 : std::stod(tokens.at(36)), tokens.at(37).empty() ? 0 : std::stod(tokens.at(37)),
        tokens.at(38).empty() ? 0 : std::stod(tokens.at(38)), tokens.at(39).empty() ? 0 : std::stod(tokens.at(39)), 0,
        0, 0, 0, 0, 0);
    std::copy(pose_cov.begin(), pose_cov.end(), odom_msg.pose.covariance.begin());
    std::copy(pose_cov.begin(), pose_cov.end(), status_msg.pose.covariance.begin());
    std::copy(vel_cov.begin(), vel_cov.end(), odom_msg.twist.covariance.begin());
    std::copy(vel_cov.begin(), vel_cov.end(), status_msg.velocity.covariance.begin());

    status_msg.version = tokens.at(40).empty() ? "UNKNOWN" : tokens.at(40);

    odometry_pub_.publish(odom_msg);
    imu_pub_.publish(imu_msg);
    status_pub_.publish(status_msg);
}

std::vector<double> FpConverter::BuildCovMat6D(double xx, double yy, double zz, double xy, double yz, double xz,
                                               double xx1, double yy1, double zz1, double xy1, double yz1, double xz1) {
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
}  // namespace fixposition