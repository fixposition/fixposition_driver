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
 * @brief Implementation of FP_A-TF parser
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
static constexpr int from_frame_idx = 5;
static constexpr int to_frame_idx = 6;
static constexpr int translation_x_idx = 7;
static constexpr int translation_y_idx = 8;
static constexpr int translation_z_idx = 9;
static constexpr int orientation_w_idx = 10;
static constexpr int orientation_x_idx = 11;
static constexpr int orientation_y_idx = 12;
static constexpr int orientation_z_idx = 13;

void FP_TF::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        WARNING_S("Error in parsing FP_A-TF string with " << tokens.size() << " fields");

    } else {
        // If size is ok, check version
        const int version = std::stoi(tokens.at(msg_version_idx));

        ok = version == kVersion_;
        if (!ok) {
            // Version is wrong
            WARNING_S("Error in parsing FP_A-TF string with version " << version << "");
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // Populate fields
    tf.stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));
    tf.frame_id = "FP_" + tokens.at(from_frame_idx);
    tf.child_frame_id = "FP_" + tokens.at(to_frame_idx);

    tf.translation =
        Vector3ToEigen(tokens.at(translation_x_idx), tokens.at(translation_y_idx), tokens.at(translation_z_idx));
    tf.rotation = Vector4ToEigen(tokens.at(orientation_w_idx), tokens.at(orientation_x_idx),
                                 tokens.at(orientation_y_idx), tokens.at(orientation_z_idx));

    // Check if TF is valid
    valid_tf = !(tf.rotation.w() == 0 && tf.rotation.vec().isZero());
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
