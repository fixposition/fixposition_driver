/**
 *  @file
 *  @brief Implementation of FP_A-GNSSCORR parser
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
static constexpr int gnss1_fix_idx = 5;
static constexpr int gnss1_nsig_l1_idx = 6;
static constexpr int gnss1_nsig_l2_idx = 7;
static constexpr int gnss2_fix_idx = 8;
static constexpr int gnss2_nsig_l1_idx = 9;
static constexpr int gnss2_nsig_l2_idx = 10;
static constexpr int corr_latency_idx = 11;
static constexpr int corr_update_rate_idx = 12;
static constexpr int corr_data_rate_idx = 13;
static constexpr int corr_msg_rate_idx = 14;
static constexpr int sta_id_idx = 15;
static constexpr int sta_lat_idx = 16;
static constexpr int sta_lon_idx = 17;
static constexpr int sta_height_idx = 18;
static constexpr int sta_dist_idx = 19;

void FP_GNSSCORR::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing FP_A-GNSSCORR string with " << tokens.size() << " fields!\n";
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing FP_A-GNSSCORR string with version " << _version << "!\n";
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // Populate VRTK message header
    stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));;

    // GNSS status data
    gnss1_fix     = ParseStatusFlag(tokens, gnss1_fix_idx);
    gnss1_nsig_l1 = StringToInt(tokens.at(gnss1_nsig_l1_idx));
    gnss1_nsig_l2 = StringToInt(tokens.at(gnss1_nsig_l2_idx));
    gnss2_fix     = ParseStatusFlag(tokens, gnss2_fix_idx);
    gnss2_nsig_l1 = StringToInt(tokens.at(gnss2_nsig_l1_idx));
    gnss2_nsig_l2 = StringToInt(tokens.at(gnss2_nsig_l2_idx));

    // Correction data status
    corr_latency     = StringToDouble(tokens.at(corr_latency_idx));
    corr_update_rate = StringToDouble(tokens.at(corr_update_rate_idx));
    corr_data_rate   = StringToDouble(tokens.at(corr_data_rate_idx));
    corr_msg_rate    = StringToDouble(tokens.at(corr_msg_rate_idx));
    sta_id     = StringToInt(tokens.at(sta_id_idx));
    sta_lat    = StringToDouble(tokens.at(sta_lon_idx));
    sta_lon    = StringToDouble(tokens.at(sta_lat_idx));
    sta_height = StringToDouble(tokens.at(sta_height_idx));
    sta_dist   = StringToInt(tokens.at(sta_dist_idx));
}

}  // namespace fixposition
