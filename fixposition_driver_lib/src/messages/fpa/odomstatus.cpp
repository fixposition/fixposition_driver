/**
 *  @file
 *  @brief Implementation of FP_A-ODOMSTATUS parser
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
static constexpr int init_status_idx = 5;
static constexpr int fusion_imu_idx = 6;
static constexpr int fusion_gnss1_idx = 7;
static constexpr int fusion_gnss2_idx = 8;
static constexpr int fusion_corr_idx = 9;
static constexpr int fusion_cam1_idx = 10;
static constexpr int reserved0_idx = 11;
static constexpr int fusion_ws_idx = 12;
static constexpr int fusion_markers_idx = 13;
static constexpr int reserved1_idx = 14;
static constexpr int reserved2_idx = 15;
static constexpr int reserved3_idx = 16;
static constexpr int reserved4_idx = 17;
static constexpr int imu_status_idx = 18;
static constexpr int imu_noise_idx = 19;
static constexpr int imu_conv_idx = 20;
static constexpr int gnss1_status_idx = 21;
static constexpr int gnss2_status_idx = 22;
static constexpr int baseline_status_idx = 23;
static constexpr int corr_status_idx = 24;
static constexpr int cam1_status_idx = 25;
static constexpr int reserved5_idx = 26;
static constexpr int ws_status_idx = 27;
static constexpr int ws_conv_idx = 28;
static constexpr int markers_status_idx = 29;
static constexpr int markers_conv_idx = 30;
static constexpr int reserved6_idx = 31;
static constexpr int reserved7_idx = 32;
static constexpr int reserved8_idx = 33;
static constexpr int reserved9_idx = 34;
static constexpr int reserved10_idx = 35;
static constexpr int reserved11_idx = 36;
static constexpr int reserved12_idx = 37;
static constexpr int reserved13_idx = 38;
static constexpr int reserved14_idx = 39;
static constexpr int reserved15_idx = 40;

void FP_ODOMSTATUS::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing FP_A-ODOMSTATUS string with " << tokens.size() << " fields!\n";
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing FP_A-ODOMSTATUS string with version " << _version << "!\n";
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
    init_status = ParseStatusFlag(tokens, init_status_idx);
    fusion_imu = ParseStatusFlag(tokens, fusion_imu_idx);
    fusion_gnss1 = ParseStatusFlag(tokens, fusion_gnss1_idx);
    fusion_gnss2 = ParseStatusFlag(tokens, fusion_gnss2_idx);
    fusion_corr = ParseStatusFlag(tokens, fusion_corr_idx);
    fusion_cam1 = ParseStatusFlag(tokens, fusion_cam1_idx);
    fusion_ws = ParseStatusFlag(tokens, fusion_ws_idx);
    fusion_markers = ParseStatusFlag(tokens, fusion_markers_idx);
    imu_status = ParseStatusFlag(tokens, imu_status_idx);
    imu_noise = ParseStatusFlag(tokens, imu_noise_idx);
    imu_conv = ParseStatusFlag(tokens, imu_conv_idx);
    gnss1_status = ParseStatusFlag(tokens, gnss1_status_idx);
    gnss2_status = ParseStatusFlag(tokens, gnss2_status_idx);
    baseline_status = ParseStatusFlag(tokens, baseline_status_idx);
    corr_status = ParseStatusFlag(tokens, corr_status_idx);
    cam1_status = ParseStatusFlag(tokens, cam1_status_idx);
    ws_status = ParseStatusFlag(tokens, ws_status_idx);
    ws_conv = ParseStatusFlag(tokens, ws_conv_idx);
    markers_status = ParseStatusFlag(tokens, markers_status_idx);
    markers_conv = ParseStatusFlag(tokens, markers_conv_idx);
}

}  // namespace fixposition
