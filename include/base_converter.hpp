/**
 * @file base_converter.hpp
 * @author Andreea Lutac (andreea.lutac@fixposition.ch)
 * @date 2020-02-25
 * @version 0.1
 *
 * @copyright @ Fixposition AG (c) 2017 - 2020
 *
 * @brief Base abstract class of output message converters
 *
 * @details
 */

#ifndef ROS_OUTPUTCONVERTER
#define ROS_OUTPUTCONVERTER

#include <nav_msgs/Odometry.h>

#include <fp_common_datatypes/datatypes.hpp>
#include <fp_common_io/format_utils.hpp>

#include "time_conversions.hpp"

class BaseConverter {
   public:
    BaseConverter() {}

    virtual nav_msgs::Odometry convert(const std::string& state) {
        nav_msgs::Odometry empty;
        return empty;
    }

    /**
     * @brief Convert GPS Week and GPS Sec in Week to ros::Time
     *
     * @param GPSWeekSec
     * @return ros::Time
     */
    static ros::Time generate_ROS_time(const GPSWeekSec& gps_t) { return GPSWeekSec2RosTime(gps_t); }

    /**
     * @brief From NMEA's Data part calculate the checksum of the data and
     * check it against the received checksum
     *
     * @return true or false if the checksums match
     */
    static bool verify_checksum(const std::string& value, unsigned int rec_checksum) {
        int len = value.size();
        unsigned int checksum = 0x00;
        for (int i = 0; i < len; ++i) {
            checksum = checksum ^ (0xff & (unsigned int)value.at(i));
        }

        return (rec_checksum == checksum);
    }

    static void split_message(const std::string& msg, const std::string& delim, std::vector<std::string>* tokens) {
        std::vector<std::string> tokens;
        size_t pos = 0;
        size_t newpos;
        while (pos != string::npos) {
            newpos = msg.find_first_of(delim, pos);
            tokens->push_back(msg.substr(pos, newpos - pos));
            if (pos != string::npos) pos++;
        }
    }
};

#endif