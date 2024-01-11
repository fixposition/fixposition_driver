/**
 *  @file
 *  @brief Implementation of GpggaConverter converter
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
#include <fixposition_driver_lib/converter/gpgga.hpp>

namespace fixposition {

/// msg field indices
static constexpr const int time_idx = 1;
static constexpr const int lat_idx = 2;
static constexpr const int lat_ns_idx = 3;
static constexpr const int lon_idx = 4;
static constexpr const int lon_ew_idx = 5;
static constexpr const int quality_idx = 6;
static constexpr const int num_sv_idx = 7;
static constexpr const int hdop_idx = 8;
static constexpr const int alt_idx = 9;
static constexpr const int alt_unit_idx = 10;
static constexpr const int sep_idx = 11;
static constexpr const int sep_unit_idx = 12;
static constexpr const int diff_age_idx = 13;
static constexpr const int diff_sta_idx = 14;

void GpggaConverter::ConvertTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        std::cout << "Error in parsing GPGGA string with " << tokens.size() << " fields! GPGGA message will be empty.\n";
        msg_ = NavSatFixData();
        return;
    }

    // Header stamps
    msg_.stamp = times::GpsTime(0, 0); // ConvertGpsTime(tokens.at(time_idx), tokens.at(time_idx));
    msg_.frame_id = "LLH";

    // LLH coordinates
    const std::string _latstr = tokens.at(lat_idx);
    double _lat = StringToDouble(_latstr.substr(0,2)) + StringToDouble((_latstr.substr(2))) / 60;
    if (tokens.at(lat_ns_idx).compare("S") == 0) _lat *= -1;
    msg_.latitude = _lat;

    const std::string _lonstr = tokens.at(lon_idx);
    double _lon = StringToDouble(_lonstr.substr(0,3)) + StringToDouble((_lonstr.substr(3))) / 60;
    if (tokens.at(lon_ew_idx).compare("W") == 0) _lon *= -1;
    msg_.longitude = _lon;

    msg_.altitude = StringToDouble(tokens.at(alt_idx));

    // Covariance diagonals
    const double hdop = StringToDouble(tokens.at(hdop_idx));
    msg_.cov(0, 0) = hdop * hdop;
    msg_.cov(1, 1) = hdop * hdop;
    msg_.cov(2, 2) = 4 * hdop * hdop;

    // Rest of covariance fields
    msg_.cov(0, 1) = msg_.cov(1, 0) = 0.0;
    msg_.cov(0, 2) = msg_.cov(2, 0) = 0.0;
    msg_.cov(1, 2) = msg_.cov(2, 1) = 0.0;
    msg_.position_covariance_type = 1; // COVARIANCE_TYPE_APPROXIMATED

    // Process all observers
    for (auto& ob : obs_) {
        ob(msg_);
    }
}

}  // namespace fixposition
