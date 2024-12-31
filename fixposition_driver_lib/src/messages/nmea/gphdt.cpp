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
 * @brief Implementation of NMEA-GP-HDT parser
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
static constexpr int heading_idx = 1;
static constexpr int true_ind_idx = 2;

void GP_HDT::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        WARNING_S("Error in parsing NMEA-GP-HDT string with " << tokens.size() << " fields");
        ResetData();
        return;
    }

    // Populate heading
    heading = StringToDouble(tokens.at(heading_idx));
    true_ind = StringToChar(tokens.at(true_ind_idx));
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
