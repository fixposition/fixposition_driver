/**
 *  @file
 *  @brief Implementation of NMEA-GP-GSV parser
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
static constexpr int sentences_idx = 1;
static constexpr int sent_num_idx = 2;
static constexpr int num_sats_idx = 3;
static constexpr int sat_id_idx = 4;
static constexpr int elev_idx = 5;
static constexpr int azim_idx = 6;
static constexpr int cno_idx = 7;
//static constexpr int signal_id_idx = 4 + 4*num_sats;

void GP_GSV::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        std::cout << "Error in parsing NMEA-GP-GSV string with " << tokens.size() << " fields!\n";
        ResetData();
        return;
    }

    // Populate fields
    sentences = StringToInt(tokens.at(sentences_idx));
    sent_num  = StringToInt(tokens.at(sent_num_idx));
    num_sats  = StringToInt(tokens.at(num_sats_idx));

    // Populate satellite information
    int offset = 0;
    for (int i = 0; i < num_sats; i++) {
        sat_id[offset] = StringToInt(tokens.at(sat_id_idx + 4*offset));
        elev[offset]   = StringToInt(tokens.at(elev_idx + 4*offset));
        azim[offset]   = StringToInt(tokens.at(azim_idx + 4*offset));
        cno[offset]    = StringToInt(tokens.at(cno_idx + 4*offset));
        offset++;
    }

    // Populate Dilution of Precision (DOP)
    signal_id = StringToChar(tokens.at(4 + 4*num_sats));
}

}  // namespace fixposition
