/**
 *  @file
 *  @brief Declaration of time conversion functions
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

#ifndef __FIXPOSITION_DRIVER_LIB_TIME_CONVERSIONS__
#define __FIXPOSITION_DRIVER_LIB_TIME_CONVERSIONS__

/* EXTERNAL */
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace BOOST_POSIX = boost::posix_time;

namespace fixposition {

namespace times {
/**
 * @brief Type for nanosecond times
 *
 */
using TIME_NS_T = uint64_t;

namespace Constants {

static constexpr int gps_leap_time_s = 18;  /// gps leap seconds

/// unix posix time begin
static const BOOST_POSIX::ptime unix_epoch_begin = BOOST_POSIX::time_from_string("1970-01-01 00:00:00.000");
/// gps time begin
static const BOOST_POSIX::ptime gps_epoch_begin = BOOST_POSIX::time_from_string("1980-01-06 00:00:00.000");
static constexpr int sec_per_week = 604800;  //!< seconds per week
static constexpr int sec_per_day = 86400;    //!< seconds per day

static constexpr double ns_to_sec = 1.0e-9;  //!< convert ns to seconds
static constexpr double sec_to_ns = 1.0e9;   //!< convert seconds to ns

}  // namespace Constants

/**
 * @brief Time formatted as GPS Week Number wno and Time of Week tow. Only time in this format
 * is GPS-Time, all other time formats are UTC.
 *
 */
class GpsTime {
   public:
    int wno;                             //!> week number
    double tow;                          //!> time of week
    static const int wno_precision = 4;  // 4 is enough because week number is around 2000;
    static const int precision = 9;
    static const int length = 10;

    /**
     * @brief Construct a new GpsTime object
     *
     */
    GpsTime() : wno(0), tow(0.0) {}

    /**
     * @brief Construct a new GpsTime object
     *
     * @param[in] week wno
     * @param[in] sec tow
     */
    GpsTime(int week, double sec) : wno(week), tow(sec) {
        int delta_week = std::floor(tow / Constants::sec_per_week);
        wno += delta_week;
        tow = tow - delta_week * Constants::sec_per_week;
    }

    GpsTime(GpsTime const&) = default;

    GpsTime& operator=(const GpsTime gws) {
        tow = gws.tow;
        wno = gws.wno;
        return *this;
    }

    GpsTime& operator+=(const double sec) {
        double gps_sec_tmp = tow + sec;
        // std::cout << gps_sec_tmp << std::endl;
        int delta_week = std::floor(gps_sec_tmp / Constants::sec_per_week);
        // std::cout << delta_week << std::endl;
        wno += delta_week;
        tow = gps_sec_tmp - delta_week * Constants::sec_per_week;
        return *this;
    }

    GpsTime& operator+=(const GpsTime in) {
        double d_sec = in.wno * Constants::sec_per_week + in.tow;
        *this += d_sec;
        return *this;
    }

    GpsTime& operator-=(const double sec) {
        *this += -sec;
        return *this;
    }

    GpsTime& operator-=(const GpsTime in) {
        double d_sec = in.wno * Constants::sec_per_week + in.tow;
        *this -= d_sec;
        return *this;
    }

    GpsTime operator+(double sec) {
        GpsTime res(*this);
        res += sec;
        return res;
    }

    GpsTime operator+(const GpsTime in) {
        GpsTime res(*this);
        res += in;
        return res;
    }

    GpsTime operator-(double sec) {
        GpsTime res(*this);
        res -= sec;
        return res;
    }

    GpsTime operator-(const GpsTime in) {
        GpsTime res(*this);
        res -= in;
        return res;
    }

    bool operator==(const GpsTime gws) { return std::abs(tow - gws.tow) < 1e-3 && wno == gws.wno; }

    bool operator>(const GpsTime gws) {
        if (wno == gws.wno) {
            return tow > gws.tow;
        } else {
            return wno > gws.wno;
        }
    }

    bool operator<(const GpsTime gws) {
        if (wno == gws.wno) {
            return tow < gws.tow;
        } else {
            return wno < gws.wno;
        }
    }

    bool operator!=(const GpsTime gws) { return !(*this == gws); }

    /**
     * @brief Operator << for use in stream
     * hard coded week size as 4 and second precision length according to class
     * member
     *
     * @param[in] stream
     * @param[in] gps_time
     * @return std::ostream&
     */
    friend std::ostream& operator<<(std::ostream& stream, const GpsTime& gps_time) {
        std::string week = std::to_string(gps_time.wno);
        week.resize(4, ' ');
        stream << week << " ";
        std::stringstream sec_ss;
        sec_ss.precision(gps_time.precision);
        sec_ss << gps_time.tow;
        std::string sec = sec_ss.str();
        if (sec.find(".") == std::string::npos) {
            sec = sec + ".";
        }
        sec.resize(gps_time.length, '0');
        stream << sec;

        return stream;
    }
};

/**
 * @brief
 * Only work after 2017.1.1
 *
 * @param[in] gps_time
 * @return BOOST_POSIX::ptime
 */
inline BOOST_POSIX::ptime GpsTimeToPtime(const GpsTime& gps_time) {
    TIME_NS_T micro_s = (gps_time.wno * Constants::sec_per_week + gps_time.tow - Constants::gps_leap_time_s) * 1e6;
    return Constants::gps_epoch_begin + BOOST_POSIX::microseconds(micro_s);
}

/**
 * @brief
 *  Only work after 2017.1.1
 *
 * @param[in] boost_ptime
 * @return GpsTime
 */
inline GpsTime PtimeToGpsTime(const BOOST_POSIX::ptime& boost_ptime) {
    BOOST_POSIX::time_duration gps_duration = boost_ptime - Constants::gps_epoch_begin;
    int weekcount = gps_duration.total_seconds() / Constants::sec_per_week;
    double sec_in_week =
        gps_duration.total_microseconds() / 1e6 - weekcount * Constants::sec_per_week + Constants::gps_leap_time_s;
    return GpsTime(weekcount, sec_in_week);
}

/**
 * @brief Convert UTC time with milliseconds to GPS time
 *
 * @param[in] utcTimeString
 * @param[in] gpsWeek
 * @param[in] gpsTimeOfWeek
 * @return GpsTime
 */
inline void convertToGPSTime(const std::string& utcTimeString, std::string& gpsWeek, std::string& gpsTimeOfWeek) {
    // Define constants
    const double secondsInWeek = 604800.0; // 7 days in seconds

    // Parse the input string
    std::tm tmTime = {};
    std::istringstream iss(utcTimeString);
    iss >> std::get_time(&tmTime, "%d/%m/%Y %H:%M:%S");

    // Read milliseconds from input
    char dot;
    std::string milliseconds;
    iss >> dot >> milliseconds;
    double ms = std::stod("0." + milliseconds);
    
    if (iss.fail()) {
        std::cerr << "Error parsing input string.\n";
        return;
    }
    
    // Convert UTC time to time since epoch
    std::time_t utcTime = std::mktime(&tmTime);

    // GPS epoch time (January 6, 1980)
    std::tm gpsEpoch = {};
    gpsEpoch.tm_year = 80; // years since 1900
    gpsEpoch.tm_mon = 0;   // months since January
    gpsEpoch.tm_mday = 6;  // day of the month
    std::time_t gpsEpochTime = std::mktime(&gpsEpoch);

    // Calculate GPS time of week and GPS week number
    double timeDifference = std::difftime(utcTime, gpsEpochTime);
    int gpsWeekNumber = static_cast<int>(std::floor(timeDifference / secondsInWeek));
    double gpsTime = std::fmod(timeDifference, secondsInWeek);

    // Add milliseconds to GPS time
    gpsTime += (Constants::gps_leap_time_s + ms);

    // Convert results to strings
    std::ostringstream ossWeek, ossTime;
    ossWeek << gpsWeekNumber;
    ossTime << std::fixed << std::setprecision(6) << gpsTime;

    gpsWeek = ossWeek.str();
    gpsTimeOfWeek = ossTime.str();
}

}  // namespace times
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_TIME_CONVERSIONS__
