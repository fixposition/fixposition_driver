/**
 *  @file
 *  @brief Implementation of GprmcConverter converter
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

/* SYSTEM / STL */
#include <iostream>

/* PACKAGE */
#include <fixposition_driver_lib/converter/gprmc.hpp>

namespace fixposition {

// unit transformation constant
static constexpr double knots_to_ms = 1852.0 / 3600.0; //!< convert knots to m/s

/// msg field indices
static constexpr const int time_idx = 1;
static constexpr const int status_idx = 2;
static constexpr const int lat_idx = 3;
static constexpr const int lat_ns_idx = 4;
static constexpr const int lon_idx = 5;
static constexpr const int lon_ew_idx = 6;
static constexpr const int speed_idx = 7;
static constexpr const int course_idx = 8;
static constexpr const int date_idx = 9;
static constexpr const int magvar_idx = 10;
static constexpr const int magvar_ew = 11;
static constexpr const int mode_idx = 12;

void GprmcConverter::ConvertTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        std::cout << "Error in parsing GPRMC string with " << tokens.size() << " fields! GPRMC message will be empty.\n";
        msg_ = GprmcData();
        return;
    }

    // Header stamps
    msg_.time = tokens.at(time_idx);

    // LLH coordinates
    const std::string _latstr = tokens.at(lat_idx);
    double _lat = StringToDouble(_latstr.substr(0,2)) + StringToDouble((_latstr.substr(2))) / 60;
    if (tokens.at(lat_ns_idx).compare("S") == 0) _lat *= -1;
    msg_.latitude = _lat;

    const std::string _lonstr = tokens.at(lon_idx);
    double _lon = StringToDouble(_lonstr.substr(0,3)) + StringToDouble((_lonstr.substr(3))) / 60;
    if (tokens.at(lon_ew_idx).compare("W") == 0) _lon *= -1;
    msg_.longitude = _lon;

    // Speed [m/s] and course [deg] over ground
    msg_.speed = StringToDouble(tokens.at(speed_idx)) * knots_to_ms;
    msg_.course = StringToDouble(tokens.at(course_idx));

    // Get GPS status
    msg_.mode = tokens.at(mode_idx);

    // Process all observers
    for (auto& ob : obs_) {
        ob(msg_);
    }
}

}  // namespace fixposition
