/**
 *  @file
 *  @brief Implementation of FP_A-TP parser
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
static constexpr int tp_name_idx = 3;
static constexpr int timebase_idx = 4;
static constexpr int timeref_idx = 5;
static constexpr int tp_tow_sec_idx = 6;
static constexpr int tp_tow_psec_idx = 7;
static constexpr int gps_leaps_idx = 8;

void FP_TP::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing FP_A-TP string with " << tokens.size() << " fields!\n";
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing FP_A-TP string with version " << _version << "!\n";
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // Populate fields
    tp_name = tokens.at(tp_name_idx);
    timebase = tokens.at(timebase_idx);
    timeref = tokens.at(timeref_idx);
    tp_tow_sec = StringToInt(tokens.at(tp_tow_sec_idx));
    tp_tow_psec = StringToDouble(tokens.at(tp_tow_psec_idx));
    gps_leaps = StringToInt(tokens.at(gps_leaps_idx));
}

}  // namespace fixposition
