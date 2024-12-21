/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Implementation of NMEA-GP-GLL parser
 */

/* LIBC/STL */

/* EXTERNAL */
#include <fpsdk_common/logging.hpp>

/* PACKAGE */
#include "fixposition_driver_lib/messages/base_converter.hpp"
#include "fixposition_driver_lib/messages/nmea_type.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

/// msg field indices
static constexpr int lat_idx = 1;
static constexpr int lat_ns_idx = 2;
static constexpr int lon_idx = 3;
static constexpr int lon_ew_idx = 4;
static constexpr int time_idx = 5;
static constexpr int status_idx = 6;
static constexpr int mode_idx = 7;

void GP_GLL::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        WARNING_S("Error in parsing NMEA-GP-GLL string with " << tokens.size() << " fields");
        ResetData();
        return;
    }

    // Populate time field
    time_str = tokens.at(time_idx);

    // LLH coordinates
    double _lat = 0.0;
    double _lon = 0.0;
    const std::string _latstr = tokens.at(lat_idx);
    const std::string _lonstr = tokens.at(lon_idx);

    if (!_latstr.empty()) {
        _lat = StringToDouble(_latstr.substr(0, 2)) + StringToDouble((_latstr.substr(2))) / 60;
        if (tokens.at(lat_ns_idx).compare("S") == 0) _lat *= -1;
    }

    if (!_lonstr.empty()) {
        _lon = StringToDouble(_lonstr.substr(0, 3)) + StringToDouble((_lonstr.substr(3))) / 60;
        if (tokens.at(lon_ew_idx).compare("W") == 0) _lon *= -1;
    }

    latlon = Eigen::Vector2d(_lat, _lon);

    // LLH indicators
    status = StringToChar(tokens.at(status_idx));
    lat_ns = StringToChar(tokens.at(lat_ns_idx));
    lon_ew = StringToChar(tokens.at(lon_ew_idx));
    mode = StringToChar(tokens.at(mode_idx));
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
