/**
 *  @file
 *  @brief Implementation of NMEA-GP-RMC parser
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

/* PACKAGE */
#include <fixposition_driver_lib/messages/nmea_type.hpp>
#include <fixposition_driver_lib/messages/base_converter.hpp>

namespace fixposition {

// unit conversion constant
static constexpr double knots_to_ms = 1852.0 / 3600.0; //!< convert knots to m/s

/// msg field indices
static constexpr int time_idx = 1;
static constexpr int status_idx = 2;
static constexpr int lat_idx = 3;
static constexpr int lat_ns_idx = 4;
static constexpr int lon_idx = 5;
static constexpr int lon_ew_idx = 6;
static constexpr int speed_idx = 7;
static constexpr int course_idx = 8;
static constexpr int date_idx = 9;
static constexpr int magvar_idx = 10;
static constexpr int magvar_ew_idx = 11;
static constexpr int mode_idx = 12;

void GP_RMC::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        std::cout << "Error in parsing NMEA-GP-RMC string with " << tokens.size() << " fields!\n";
        ResetData();
        return;
    }

    // Check that critical message fields are populated
    for (int i = 1; i < 9; i++) {
        if (tokens.at(i).empty()) {
            ResetData();
            return;
        }
    }

    // Time and date strings
    date_str = tokens.at(date_idx);
    time_str = tokens.at(time_idx);

    // LLH coordinates
    const std::string _latstr = tokens.at(lat_idx);
    double _lat = StringToDouble(_latstr.substr(0,2)) + StringToDouble((_latstr.substr(2))) / 60;
    if (tokens.at(lat_ns_idx).compare("S") == 0) _lat *= -1;

    const std::string _lonstr = tokens.at(lon_idx);
    double _lon = StringToDouble(_lonstr.substr(0,3)) + StringToDouble((_lonstr.substr(3))) / 60;
    if (tokens.at(lon_ew_idx).compare("W") == 0) _lon *= -1;

    latlon = Eigen::Vector2d(_lat, _lon);

    // LLH indicators
    status = StringToChar(tokens.at(status_idx));
    lat_ns = StringToChar(tokens.at(lat_ns_idx));
    lon_ew = StringToChar(tokens.at(lon_ew_idx));
    mode   = StringToChar(tokens.at(mode_idx));

    // Speed and course over ground
    speed    = StringToDouble(tokens.at(speed_idx));
    speed_ms = speed * knots_to_ms;
    course   = StringToDouble(tokens.at(course_idx));
}

}  // namespace fixposition
