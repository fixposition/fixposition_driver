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
nav_msgs::Odometry FpMsgConverter::convert(const std::string& state) {
    nav_msgs::Odometry msg;

    if (!state.empty()) {
        std::size_t star = state.find_last_of("*");
        std::string state_data = state.substr(0, star);
        state_data = state_data.erase(0, 1);

        unsigned int checksum = std::stoul(state.substr(star + 1, 2), nullptr, 16);
        bool ret = verify_checksum(state_data, checksum);
        if (!ret) {
            ROS_DEBUG_STREAM("Checksum invalid! Odometry message will be empty.");
            return msg;
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
            return msg;
        }
        if (!tokens[1].empty() && !tokens[2].empty()) {
            GPSWeekSec weeksec(std::stoi(tokens[1]), std::stod(tokens[2]));
            msg.header.stamp = GPSWeekSec2RosTime(weeksec);
        } else {
            ROS_DEBUG_STREAM("GPS time empty. Replacing with current ROS time.");
            msg.header.stamp = ros::Time::now();
        }

        msg.pose.pose.position.x = tokens[3].empty() ? 0 : std::stod(tokens[3]);
        msg.pose.pose.position.y = tokens[4].empty() ? 0 : std::stod(tokens[4]);
        msg.pose.pose.position.z = tokens[5].empty() ? 0 : std::stod(tokens[5]);

        msg.twist.twist.linear.x = tokens[6].empty() ? 0 : std::stod(tokens[6]);
        msg.twist.twist.linear.y = tokens[7].empty() ? 0 : std::stod(tokens[7]);
        msg.twist.twist.linear.z = tokens[8].empty() ? 0 : std::stod(tokens[8]);

        msg.pose.pose.orientation.w = tokens[9].empty() ? 0 : std::stod(tokens[9]);
        msg.pose.pose.orientation.x = tokens[10].empty() ? 0 : std::stod(tokens[10]);
        msg.pose.pose.orientation.y = tokens[11].empty() ? 0 : std::stod(tokens[11]);
        msg.pose.pose.orientation.z = tokens[12].empty() ? 0 : std::stod(tokens[12]);
    } else {
        ROS_ERROR_STREAM("State message empty! Not well parsed?");
    }

    return msg;
}
