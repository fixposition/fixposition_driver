/**
 *  @file
 *  @brief Implementation of NMEA-GP-VTG parser
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
#include <fixposition_driver_lib/messages/nmea_type.hpp>
#include <fixposition_driver_lib/messages/base_converter.hpp>

namespace fixposition {

/// msg field indices
static constexpr int cog_true_idx = 1;
static constexpr int cog_ref_t_idx = 2;
static constexpr int cog_mag_idx = 3;
static constexpr int cog_ref_m_idx = 4;
static constexpr int sog_knot_idx = 5;
static constexpr int sog_unit_n_idx = 6;
static constexpr int sog_kph_idx = 7;
static constexpr int sog_unit_k_idx = 8;
static constexpr int mode_idx = 9;

void GP_VTG::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        std::cout << "Error in parsing NMEA-GP-VTG string with " << tokens.size() << " fields!\n";
        ResetData();
        return;
    }

    // Populate COG
    cog_true = StringToDouble(tokens.at(cog_true_idx));
    cog_ref_t = StringToChar(tokens.at(cog_ref_t_idx));
    cog_mag = StringToDouble(tokens.at(cog_mag_idx));
    cog_ref_m = StringToChar(tokens.at(cog_ref_m_idx));

    // Populate SOG
    sog_knot = StringToDouble(tokens.at(sog_knot_idx));
    sog_unit_n = StringToChar(tokens.at(sog_unit_n_idx));
    sog_kph = StringToDouble(tokens.at(sog_kph_idx));
    sog_unit_k = StringToChar(tokens.at(sog_unit_k_idx));

    // Populate mode
    mode = StringToChar(tokens.at(mode_idx));
}

}  // namespace fixposition
