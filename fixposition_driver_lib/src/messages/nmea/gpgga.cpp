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
#include <fixposition_driver_lib/messages/nmea_type.hpp>
#include <fixposition_driver_lib/messages/base_converter.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

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

void GP_GGA::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        std::cout << "Error in parsing GPGGA string with " << tokens.size() << " fields! GPGGA message will be empty.\n";
        ResetData();
        return;
    }

    // Check that critical message fields are populated
    for (int i = 1; i < 11; i++) {
        if (tokens.at(i).empty()) {
            ResetData();
            return;
        }
    }

    // Time string
    time_str = tokens.at(time_idx);

    // LLH coordinates
    const std::string _latstr = tokens.at(lat_idx);
    double _lat = StringToDouble(_latstr.substr(0,2)) + StringToDouble((_latstr.substr(2))) / 60;
    if (tokens.at(lat_ns_idx).compare("S") == 0) _lat *= -1;

    const std::string _lonstr = tokens.at(lon_idx);
    double _lon = StringToDouble(_lonstr.substr(0,3)) + StringToDouble((_lonstr.substr(3))) / 60;
    if (tokens.at(lon_ew_idx).compare("W") == 0) _lon *= -1;

    llh = Eigen::Vector3d(_lat, _lon, StringToDouble(tokens.at(alt_idx)));

    // LLH indicators
    lat_ns   = tokens.at(lat_ns_idx)[0];
    lon_ew   = tokens.at(lon_ew_idx)[0];
    alt_unit = tokens.at(alt_unit_idx)[0];
    quality  = StringToDouble(tokens.at(quality_idx));
    num_sv   = StringToDouble(tokens.at(num_sv_idx));

    // Dilution of precision
    hdop = StringToDouble(tokens.at(hdop_idx));

    // // Covariance diagonals
    // msg_.cov(0, 0) = hdop * hdop;
    // msg_.cov(1, 1) = hdop * hdop;
    // msg_.cov(2, 2) = 4 * hdop * hdop;

    // // Rest of covariance fields
    // msg_.cov(0, 1) = msg_.cov(1, 0) = 0.0;
    // msg_.cov(0, 2) = msg_.cov(2, 0) = 0.0;
    // msg_.cov(1, 2) = msg_.cov(2, 1) = 0.0;
    // msg_.position_covariance_type = 1; // COVARIANCE_TYPE_APPROXIMATED

    // Differental data
    diff_age = StringToDouble(tokens.at(diff_age_idx));
    diff_sta = tokens.at(diff_sta_idx);
}

}  // namespace fixposition
