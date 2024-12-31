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
 * @brief Implementation of NMEA-GP-GST parser
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
static constexpr int time_idx = 1;
static constexpr int rms_range_idx = 2;
static constexpr int std_major_idx = 3;
static constexpr int std_minor_idx = 4;
static constexpr int angle_major_idx = 5;
static constexpr int std_lat_idx = 6;
static constexpr int std_lon_idx = 7;
static constexpr int std_alt_idx = 8;

void GP_GST::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        WARNING_S("Error in parsing NMEA-GP-GST string with " << tokens.size() << " fields");
        ResetData();
        return;
    }

    // Populate time field
    time_str = tokens.at(time_idx);

    // Standard deviation of error ellipse
    rms_range = StringToDouble(tokens.at(rms_range_idx));
    std_major = StringToDouble(tokens.at(std_major_idx));
    std_minor = StringToDouble(tokens.at(std_minor_idx));
    angle_major = StringToDouble(tokens.at(angle_major_idx));

    // Standard deviation of LLH coordinates
    std_lat = StringToDouble(tokens.at(std_lat_idx));
    std_lon = StringToDouble(tokens.at(std_lon_idx));
    std_alt = StringToDouble(tokens.at(std_alt_idx));
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
