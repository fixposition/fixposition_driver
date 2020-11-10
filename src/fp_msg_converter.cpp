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

namespace fp_common {
namespace ros_io {

FpMsgConverter::FpMsgConverter() : BaseConverter() {  // load imu antenna extrinsics
}
nav_msgs::Odometry FpMsgConverter::convert(const std::string& state) {
    nav_msgs::Odometry msg;

    if (!state.empty()) {
        std::size_t star = state.find_last_of("*");
        std::string state_data = state.substr(0, star);
        std::string state_data = state_data.erase(0);

        unsigned int checksum = std::stoul(state.substr(star + 1, 2), nullptr, 16);
        bool ret = verify_checksum(state_data, checksum);
        if (!ret) {
            ROS_ERROR_STREAM("Checksum invalid! Odometry message will be empty.");
            return msg;
        }

        std::vector<std::string> tokens;
        split_message(state_data, ",", &tokens);

        if (tokens.size() != 14) {
            ROS_ERROR_STREAM("Error in parsing! Odometry message will be empty.");
            return msg;
        }
        GPSWeekSec weeksec(std::stoi(tokens[1]), std::stod(tokens[2]));
        msg.header.stamp = GPSWeekSec2RosTime(weeksec);
        msg.pose.pose.position.x = if (tokens[3].empty()) 0 : std::stod(tokens[3]);
        msg.pose.pose.position.y = if (tokens[4].empty()) 0 : std::stod(tokens[4]);
        msg.pose.pose.position.z = if (tokens[5].empty()) 0 : std::stod(tokens[5]);

        msg.twist.twist.linear.x = if (tokens[6].empty()) 0 : std::stod(tokens[6]);
        msg.twist.twist.linear.y = if (tokens[7].empty()) 0 : std::stod(tokens[7]);
        msg.twist.twist.linear.z = if (tokens[8].empty()) 0 : std::stod(tokens[8]);

        msg.pose.pose.orientation.w = if (tokens[9].empty()) 0 : std::stod(tokens[9]);
        msg.pose.pose.orientation.x = if (tokens[10].empty()) 0 : std::stod(tokens[10]);
        msg.pose.pose.orientation.y = if (tokens[11].empty()) 0 : std::stod(tokens[11]);
        msg.pose.pose.orientation.z = if (tokens[12].empty()) 0 : std::stod(tokens[12]);
    } else {
        ROS_ERROR_STREAM("State message empty!");
    }

    return msg;
}
