/**
 *  @file
 *  @brief Implementation of Base Converter to define the interfaces
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* PACKAGE */
#include <fixposition_driver/converter/base_converter.hpp>
#include <fixposition_driver/helper.hpp>

namespace fixposition {
void BaseConverter::ConvertStringAndPublish(const std::string& in_string) {
    if (!in_string.empty()) {
        std::vector<std::string> tokens;
        SplitMessage(tokens, in_string, ",");

        if (CheckHeaderAndVersion(tokens[1], tokens[2])) {
            ROS_DEBUG_STREAM("Received " << tokens[1] << "," << tokens[2] << "  message.");
            ConvertTokensAndPublish(tokens);
        } else {
            ROS_DEBUG_STREAM("Unknown message! Type: " << tokens[1] << " Version: " << tokens[2]);
            return;
        }

    } else {
        ROS_ERROR_STREAM("State message empty! Not well parsed?");
    }
}

}  // namespace fixposition