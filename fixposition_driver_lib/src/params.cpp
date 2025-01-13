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
 * @brief Parameters
 */

/* LIBC/STL */
#include <algorithm>

/* EXTERNAL */

/* PACKAGE */
#include "fixposition_driver_lib/params.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

bool DriverParams::MessageEnabled(const std::string& message_name) const {
    return std::find(messages_.cbegin(), messages_.cend(), message_name) != messages_.end();
}

// ---------------------------------------------------------------------------------------------------------------------

bool StrToEpoch(const std::string& str, fpsdk::common::parser::fpa::FpaEpoch& epoch) {
    // clang-format off
    if      (str == "GNSS1")  { epoch = fpsdk::common::parser::fpa::FpaEpoch::GNSS1;       }
    else if (str == "GNSS2")  { epoch = fpsdk::common::parser::fpa::FpaEpoch::GNSS2;       }
    else if (str == "GNSS")   { epoch = fpsdk::common::parser::fpa::FpaEpoch::GNSS;        }
    else if (str == "FUSION") { epoch = fpsdk::common::parser::fpa::FpaEpoch::FUSION;      }
    else if (str.empty())     { epoch = fpsdk::common::parser::fpa::FpaEpoch::UNSPECIFIED; }
    // clang-format on
    else {
        return false;
    }
    return true;
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
