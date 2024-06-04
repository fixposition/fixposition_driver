/**
 *  @file
 *  @brief Implementation of FP_A-TF parser
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
#include <fixposition_driver_lib/messages/fpa_type.hpp>
#include <fixposition_driver_lib/messages/base_converter.hpp>

namespace fixposition {

/// msg field indices
static constexpr const int msg_type_idx = 1;
static constexpr const int msg_version_idx = 2;
static constexpr const int gps_week_idx = 3;
static constexpr const int gps_tow_idx = 4;
static constexpr const int from_frame_idx = 5;
static constexpr const int to_frame_idx = 6;
static constexpr const int translation_x_idx = 7;
static constexpr const int translation_y_idx = 8;
static constexpr const int translation_z_idx = 9;
static constexpr const int orientation_w_idx = 10;
static constexpr const int orientation_x_idx = 11;
static constexpr const int orientation_y_idx = 12;
static constexpr const int orientation_z_idx = 13;

void FP_TF::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing TF string with " << tokens.size() << " fields! TF will be empty.\n";

    } else {
        // If size is ok, check version
        const int version = std::stoi(tokens.at(msg_version_idx));

        ok = version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing TF string with verion " << version << " ! TF will be empty.\n";
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // header stamps
    tf.stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));
    tf.frame_id = "FP_" + tokens.at(from_frame_idx);
    tf.child_frame_id = "FP_" + tokens.at(to_frame_idx);

    tf.translation = Vector3ToEigen(tokens.at(translation_x_idx), 
                                    tokens.at(translation_y_idx),
                                    tokens.at(translation_z_idx));
    tf.rotation = Vector4ToEigen(tokens.at(orientation_w_idx), tokens.at(orientation_x_idx),
                                 tokens.at(orientation_y_idx), tokens.at(orientation_z_idx));
}

}  // namespace fixposition
