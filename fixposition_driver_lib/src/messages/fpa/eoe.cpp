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
 * @brief Implementation of FP_A-EOE parser
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
static constexpr int epoch_idx = 5;

void FP_EOE::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        WARNING_S("Error in parsing FP_A-EOE string with " << tokens.size() << " fields");
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            WARNING_S("Error in parsing FP_A-EOE string with version " << _version << "");
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // Parse time
    stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));

    // Populate fields
    epoch = tokens.at(epoch_idx);
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
