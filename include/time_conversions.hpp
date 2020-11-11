/**
 * @file time_conversions.hpp
 * @author Andreea Lutac (andreea.lutac@fixposition.ch)
 * @brief
 * version 0.1
 * @date 2020-11-11
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef TIME_CONVERSIONS
#define TIME_CONVERSIONS

#include <ros/duration.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <boost/date_time/posix_time/posix_time.hpp>
using namespace boost::posix_time;

#define GPS_LEAP_TIME_S \
    18  // need to be incremented by 1 when another leap second is introduced in
        // UTC
#define SECONDS_PER_WEEK 604800
#define SECONDS_PER_DAY 86400
static const std::string unix_epoch_begin = "1970-01-01 00:00:00.000";
static const std::string gps_epoch_begin = "1980-01-06 00:00:00.000";

/**
 * @brief Time formatted as GPS week + seconds in week. Only time in this format
 * ist GPS-Time, all other time formats are UTC.
 *
 */
class GPSWeekSec {
   public:
    int gps_week;
    double gps_sec;
    static const int week_precision = 4;  // 4 is enough because week number is around 2000;
    static const int precision = 9;
    static const int length = 10;

    /**
     * @brief Construct a new GPSWeekSec object
     *
     */
    GPSWeekSec() {}

    /**
     * @brief Construct a new GPSWeekSec object
     *
     * @param week gps_week
     * @param sec gps_sec
     */
    GPSWeekSec(int week, double sec) : gps_week(week), gps_sec(sec) {
        int delta_week = std::floor(gps_sec / SECONDS_PER_WEEK);
        gps_week += delta_week;
        gps_sec = gps_sec - delta_week * SECONDS_PER_WEEK;
    }

    GPSWeekSec &operator=(const GPSWeekSec gws) {
        gps_sec = gws.gps_sec;
        gps_week = gws.gps_week;
        return *this;
    }

    GPSWeekSec &operator+=(const double sec) {
        double gps_sec_tmp = gps_sec + sec;
        // std::cout << gps_sec_tmp << std::endl;
        int delta_week = std::floor(gps_sec_tmp / SECONDS_PER_WEEK);
        // std::cout << delta_week << std::endl;
        gps_week += delta_week;
        gps_sec = gps_sec_tmp - delta_week * SECONDS_PER_WEEK;
        return *this;
    }

    GPSWeekSec &operator+=(const GPSWeekSec in) {
        double d_sec = in.gps_week * SECONDS_PER_WEEK + in.gps_sec;
        *this += d_sec;
        return *this;
    }

    GPSWeekSec &operator-=(const double sec) {
        *this += -sec;
        return *this;
    }

    GPSWeekSec &operator-=(const GPSWeekSec in) {
        double d_sec = in.gps_week * SECONDS_PER_WEEK + in.gps_sec;
        *this -= d_sec;
        return *this;
    }

    GPSWeekSec operator+(double sec) {
        GPSWeekSec res(*this);
        res += sec;
        return res;
    }

    GPSWeekSec operator+(const GPSWeekSec in) {
        GPSWeekSec res(*this);
        res += in;
        return res;
    }

    GPSWeekSec operator-(double sec) {
        GPSWeekSec res(*this);
        res -= sec;
        return res;
    }

    GPSWeekSec operator-(const GPSWeekSec in) {
        GPSWeekSec res(*this);
        res -= in;
        return res;
    }

    bool operator==(const GPSWeekSec gws) { return abs(gps_sec - gws.gps_sec) < 1e-3 && gps_week == gws.gps_week; }

    bool operator>(const GPSWeekSec gws) {
        if (gps_week == gws.gps_week) {
            return gps_sec > gws.gps_sec;
        } else {
            return gps_week > gws.gps_week;
        }
    }

    bool operator<(const GPSWeekSec gws) {
        if (gps_week == gws.gps_week) {
            return gps_sec < gws.gps_sec;
        } else {
            return gps_week < gws.gps_week;
        }
    }

    bool operator!=(const GPSWeekSec gws) { return !(*this == gws); }

    /**
     * @brief Operator << for use in stream
     * hard coded week size as 4 and second precision length according to class
     * member
     *
     * @param stream
     * @param gps_week_sec
     * @return std::ostream&
     */
    friend std::ostream &operator<<(std::ostream &stream, const GPSWeekSec &gps_week_sec) {
        std::string week = std::to_string(gps_week_sec.gps_week);
        week.resize(4, ' ');
        stream << week << " ";
        std::stringstream sec_ss;
        sec_ss.precision(gps_week_sec.precision);
        sec_ss << gps_week_sec.gps_sec;
        std::string sec = sec_ss.str();
        if (sec.find(".") == std::string::npos) {
            sec = sec + ".";
        }
        sec.resize(gps_week_sec.length, '0');
        stream << sec;

        return stream;
    }
};

/**
 * @brief
 * Only works after 2017.1.1
 *
 * @param gps_time
 * @return ptime
 */
static ptime GPSWeekSec2Ptime(GPSWeekSec gps_time) {
    unsigned long long micro_s = (gps_time.gps_week * SECONDS_PER_WEEK + gps_time.gps_sec - GPS_LEAP_TIME_S) * 1e6;
    ptime gps_start = time_from_string(gps_epoch_begin);
    return gps_start + microseconds(micro_s);
}
static ros::Time GPSWeekSec2RosTime(GPSWeekSec input) { return ros::Time::fromBoost(GPSWeekSec2Ptime(input)); }

#endif