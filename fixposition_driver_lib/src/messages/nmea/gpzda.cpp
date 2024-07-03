/**
 *  @file
 *  @brief Implementation of NMEA-GP-ZDA parser
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

/// msg field indices
static constexpr int time_idx = 1;
static constexpr int day_idx = 2;
static constexpr int month_idx = 3;
static constexpr int year_idx = 4;
static constexpr int local_hr_idx = 5;
static constexpr int local_min_idx = 6;

void GP_ZDA::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        std::cout << "Error in parsing NMEA-GP-ZDA string with " << tokens.size() << " fields!\n";
        ResetData();
        return;
    }

    // Check that critical message fields are populated
    for (int i = 1; i < 6; i++) {
        if (tokens.at(i).empty()) {
            ResetData();
            return;
        }
    }

    // Populate time fields
    time_str = tokens.at(time_idx);
    date_str = tokens.at(day_idx) + '/' + tokens.at(month_idx) + '/' + tokens.at(year_idx);

    // Generate GPS timestamp
    if (!time_str.empty()) {
        std::string utcTimeString = date_str + " " + time_str.substr(0,2) + ":" + time_str.substr(2,2) + ":" + time_str.substr(4);
        std::string gps_tow, gps_week;
        times::convertToGPSTime(utcTimeString, gps_week, gps_tow);
        stamp = ConvertGpsTime(gps_week, gps_tow);
    }

    // Get local time
    local_hr  = StringToInt(tokens.at(local_hr_idx));
    local_min = StringToInt(tokens.at(local_min_idx));
}

}  // namespace fixposition
