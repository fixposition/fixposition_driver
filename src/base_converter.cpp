/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /
 *   /  /\  \
 *  /__/  \__\  Fixposition AG
 *
 * @file fp_msg_converter.cpp
 * @author Kailin Huang (kailin.huang@fixposition.com)
 * @brief
 * @date 2022-01-26
 *
 */

/* EXTERNAL */

/* ROS */

/* PACKAGE */
#include <fixposition_driver/converter/base_converter.hpp>
#include <fixposition_driver/helper.hpp>

namespace fixposition {
void BaseConverter::ConvertStringAndPublish(const std::string& state) {
    if (!state.empty()) {
        std::vector<std::string> tokens;
        split_message(tokens, state, ",");

        if (CheckHeader(tokens[0])) {
            ROS_DEBUG_STREAM("Received " << tokens[0] << "  message.");
            ConvertTokensAndPublish(tokens);
        } else {
            ROS_DEBUG_STREAM("Unknown message type.");
            return;
        }

    } else {
        ROS_ERROR_STREAM("State message empty! Not well parsed?");
    }
}

}  // namespace fixposition