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
 * @brief Implementation of FP_A-GNSSANT parser
 */

/* LIBC/STL */

/* EXTERNAL */
#include <fpsdk_common/logging.hpp>

/* PACKAGE */
#include "fixposition_driver_lib/messages/base_converter.hpp"
#include "fixposition_driver_lib/messages/fpa_type.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

/// msg field indices
static constexpr int msg_type_idx = 1;
static constexpr int msg_version_idx = 2;
static constexpr int gps_week_idx = 3;
static constexpr int gps_tow_idx = 4;
static constexpr int gnss1_state_idx = 5;
static constexpr int gnss1_power_idx = 6;
static constexpr int gnss1_age_idx = 7;
static constexpr int gnss2_state_idx = 8;
static constexpr int gnss2_power_idx = 9;
static constexpr int gnss2_age_idx = 10;

void FP_GNSSANT::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        WARNING_S("Error in parsing FP_A-GNSSANT string with " << tokens.size() << " fields");
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            WARNING_S("Error in parsing FP_A-GNSSANT string with version " << _version << "");
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // Populate VRTK message header
    stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));

    // GNSS status data
    gnss1_state = tokens.at(gnss1_state_idx);
    gnss1_power = tokens.at(gnss1_power_idx);
    gnss1_age = StringToInt(tokens.at(gnss1_age_idx));
    gnss2_state = tokens.at(gnss2_state_idx);
    gnss2_power = tokens.at(gnss2_power_idx);
    gnss2_age = StringToInt(tokens.at(gnss2_age_idx));
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
