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
void FpMsgConverter::convertAndPublish(const std::string& state, ros::Publisher odometry_pub,
                                       ros::Publisher status_pub) {
    nav_msgs::Odometry odom_msg;
    fixposition_output::VRTK status_msg;

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

        if (tokens.size() != 15) {
            ROS_DEBUG_STREAM("Error in parsing! Odometry message will be empty.");
            return;
        }
        if (!tokens[1].empty() && !tokens[2].empty()) {
            GPSWeekSec weeksec(std::stoi(tokens[1]), std::stod(tokens[2]));
            odom_msg.header.stamp = GPSWeekSec2RosTime(weeksec);
            status_msg.header.stamp = GPSWeekSec2RosTime(weeksec);

        } else {
            ROS_DEBUG_STREAM("GPS time empty. Replacing with current ROS time.");
            odom_msg.header.stamp = ros::Time::now();
            status_msg.header.stamp = ros::Time::now();
        }
        odom_msg.header.frame_id = status_msg.header.frame_id = "ECEF";
        odom_msg.child_frame_id = status_msg.kin_frame = "ECEF";

        odom_msg.pose.pose.position.x = status_msg.pose.pose.position.x = tokens[3].empty() ? 0 : std::stod(tokens[3]);
        odom_msg.pose.pose.position.y = status_msg.pose.pose.position.y = tokens[4].empty() ? 0 : std::stod(tokens[4]);
        odom_msg.pose.pose.position.z = status_msg.pose.pose.position.z = tokens[5].empty() ? 0 : std::stod(tokens[5]);

        odom_msg.twist.twist.linear.x = status_msg.velocity.twist.linear.x =
            tokens[6].empty() ? 0 : std::stod(tokens[6]);
        odom_msg.twist.twist.linear.y = status_msg.velocity.twist.linear.y =
            tokens[7].empty() ? 0 : std::stod(tokens[7]);
        odom_msg.twist.twist.linear.z = status_msg.velocity.twist.linear.z =
            tokens[8].empty() ? 0 : std::stod(tokens[8]);

        odom_msg.pose.pose.orientation.w = status_msg.pose.pose.orientation.w =
            tokens[9].empty() ? 0 : std::stod(tokens[9]);
        odom_msg.pose.pose.orientation.x = status_msg.pose.pose.orientation.x =
            tokens[10].empty() ? 0 : std::stod(tokens[10]);
        odom_msg.pose.pose.orientation.y = status_msg.pose.pose.orientation.y =
            tokens[11].empty() ? 0 : std::stod(tokens[11]);
        odom_msg.pose.pose.orientation.z = status_msg.pose.pose.orientation.z =
            tokens[12].empty() ? 0 : std::stod(tokens[12]);
        status_msg.fusion_status = tokens[13].size() == 2 ? tokens[13][0] - '0' : 0;
        status_msg.gnss_status = tokens[13].size() == 2 ? tokens[13][1] - '0' : 0;
        odometry_pub.publish(odom_msg);
        status_pub.publish(status_msg);

    } else {
        ROS_ERROR_STREAM("State message empty! Not well parsed?");
    }
}