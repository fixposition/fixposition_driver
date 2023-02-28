/**
 *  @file
 *  @brief Implementation of LlhConverter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* PACKAGE */
#include <fixposition_driver_lib/converter/llh.hpp>

namespace fixposition {

/// msg field indices
static constexpr const int msg_type_idx = 1;
static constexpr const int msg_version_idx = 2;
static constexpr const int gps_week_idx = 3;
static constexpr const int gps_tow_idx = 4;
static constexpr const int latitude_idx = 5;
static constexpr const int longitude_idx = 6;
static constexpr const int height_idx = 7;
static constexpr const int pos_cov_ee_idx = 8;
static constexpr const int pos_cov_nn_idx = 9;
static constexpr const int pos_cov_uu_idx = 10;
static constexpr const int pos_cov_en_idx = 11;
static constexpr const int pos_cov_nu_idx = 12;
static constexpr const int pos_cov_eu_idx = 13;

void LlhConverter::ConvertTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        std::cout << "Error in parsing LLH string with " << tokens.size()
                  << " fields! NavSatFix message will be empty.\n";

    } else {
        // If size is ok, check version
        const int version = std::stoi(tokens.at(msg_version_idx));

        ok = version == kVersion_;
        if (!ok) {
            // Version is wrong
            std::cout << "Error in parsing LLH string with verion " << version
                      << " ! NavSatFix message will be empty.\n";
        }
    }

    if (!ok) {
        // Reset message and return
        msg_ = NavSatFixData();
        return;
    }

    // header stamps
    msg_.stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));

    msg_.frame_id = "POI";
    msg_.latitude = StringToDouble(tokens.at(latitude_idx));
    msg_.longitude = StringToDouble(tokens.at(longitude_idx));
    msg_.altitude = StringToDouble(tokens.at(height_idx));

    // Covariance diagonals
    msg_.cov(0, 0) = StringToDouble(tokens.at(pos_cov_ee_idx));
    msg_.cov(1, 1) = StringToDouble(tokens.at(pos_cov_nn_idx));
    msg_.cov(2, 2) = StringToDouble(tokens.at(pos_cov_uu_idx));

    // Rest of covariance fields
    msg_.cov(0, 1) = msg_.cov(1, 0) = StringToDouble(tokens.at(pos_cov_en_idx));
    msg_.cov(0, 2) = msg_.cov(2, 0) = StringToDouble(tokens.at(pos_cov_nu_idx));
    msg_.cov(1, 2) = msg_.cov(2, 1) = StringToDouble(tokens.at(pos_cov_eu_idx));
    msg_.position_covariance_type = 3;

    // process all observers
    for (auto& ob : obs_) {
        ob(msg_);
    }
}

}  // namespace fixposition
