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
 * @brief Implementation of FP_A-TEXT parser
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
static constexpr int level_idx = 3;
static constexpr int text_idx = 4;

void FP_TEXT::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        WARNING_S("Error in parsing FP_A-TEXT string with " << tokens.size() << " fields");
    } else {
        // If size is ok, check version
        const int version = std::stoi(tokens.at(msg_version_idx));

        ok = version == kVersion_;
        if (!ok) {
            // Version is wrong
            WARNING_S("Error in parsing FP_A-TEXT string with version " << version << "");
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // Populate fields
    level = tokens.at(level_idx);
    text = tokens.at(text_idx);
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
