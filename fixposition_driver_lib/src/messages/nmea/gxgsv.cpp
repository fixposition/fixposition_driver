/**
 *  @file
 *  @brief Implementation of NMEA-GX-GSV parser
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
//static constexpr int signal_id_idx = 4 + (4*num_sats);

void GX_GSV::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Reset vectors
    ResetData();
    
    // Populate fields
    sentences = StringToInt(tokens.at(sentences_idx));
    sent_num = StringToInt(tokens.at(sent_num_idx));
    num_sats = StringToInt(tokens.at(num_sats_idx));

    // Populate satellite information
    unsigned int offset = 0;
    for (unsigned int i = 3; i < (tokens.size() - 2); i+=4) {
        sat_id.push_back(StringToInt(tokens.at(sat_id_idx + 4*offset)));
        elev.push_back(StringToInt(tokens.at(elev_idx + 4*offset)));
        azim.push_back(StringToInt(tokens.at(azim_idx + 4*offset)));
        cno.push_back(StringToInt(tokens.at(cno_idx + 4*offset)));
        offset++;
    }

    signal_id = tokens.back();
}

}  // namespace fixposition
