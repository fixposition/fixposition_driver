/**
 *  @file
 *  @brief Implementation of NMEA-GP-HDT parser
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
static constexpr int heading_idx = 1;
static constexpr int true_ind_idx = 2;

void GP_HDT::ConvertFromTokens(const std::vector<std::string>& tokens) {
    // Check if message size is wrong
    bool ok = tokens.size() == kSize_;
    if (!ok) {
        std::cout << "Error in parsing NMEA-GP-HDT string with " << tokens.size() << " fields!\n";
        ResetData();
        return;
    }

    // Populate heading
    heading = StringToDouble(tokens.at(heading_idx));
    true_ind = StringToChar(tokens.at(true_ind_idx));
}

}  // namespace fixposition
