/**
 *  @file
 *  @brief Implementation of FP_A-EOE parser
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
#include <fixposition_driver_lib/messages/fpa_type.hpp>
#include <fixposition_driver_lib/messages/base_converter.hpp>

namespace fixposition {

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
        std::cout << "Error in parsing FP_A-EOE string with " << tokens.size() << " fields!\n";
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing FP_A-EOE string with version " << _version << "!\n";
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

}  // namespace fixposition
