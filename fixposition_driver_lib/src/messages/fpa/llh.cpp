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
 * @brief Implementation of FP_A-LLH parser
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
static constexpr int latitude_idx = 5;
static constexpr int longitude_idx = 6;
static constexpr int height_idx = 7;
static constexpr int pos_cov_ee_idx = 8;
static constexpr int pos_cov_nn_idx = 9;
static constexpr int pos_cov_uu_idx = 10;
static constexpr int pos_cov_en_idx = 11;
static constexpr int pos_cov_nu_idx = 12;
static constexpr int pos_cov_eu_idx = 13;

void FP_LLH::ConvertFromTokens(const std::vector<std::string>& tokens) {
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        // Size is wrong
        WARNING_S("Error in parsing FP_A-LLH string with " << tokens.size() << " fields");
    } else {
        // If size is ok, check version
        const int _version = std::stoi(tokens.at(msg_version_idx));

        ok = _version == kVersion_;
        if (!ok) {
            // Version is wrong
            WARNING_S("Error in parsing FP_A-LLH string with version " << _version << "");
        }
    }

    if (!ok) {
        // Reset message and return
        ResetData();
        return;
    }

    // Populate VRTK message header
    stamp = ConvertGpsTime(tokens.at(gps_week_idx), tokens.at(gps_tow_idx));

    // LLH position
    llh = Vector3ToEigen(tokens.at(latitude_idx), tokens.at(longitude_idx), tokens.at(height_idx));

    // Covariance
    cov = BuildCovMat3D(StringToDouble(tokens.at(pos_cov_ee_idx)), StringToDouble(tokens.at(pos_cov_nn_idx)),
                        StringToDouble(tokens.at(pos_cov_uu_idx)), StringToDouble(tokens.at(pos_cov_en_idx)),
                        StringToDouble(tokens.at(pos_cov_nu_idx)), StringToDouble(tokens.at(pos_cov_eu_idx)));
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
