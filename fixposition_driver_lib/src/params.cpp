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

bool SensorParams::MessageEnabled(const std::string& message_name) const {
    return std::find(messages_.cbegin(), messages_.cend(), message_name) != messages_.end();
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
