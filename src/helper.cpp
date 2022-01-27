/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /
 *   /  /\  \
 *  /__/  \__\  Fixposition AG
 *
 * @file helper.cpp
 * @author Kailin Huang (kailin.huang@fixposition.com)
 * @brief
 * @date 2022-01-27
 *
 */

#include <fixposition_driver/helper.hpp>

namespace fixposition {

void split_message(std::vector<std::string>& tokens, const std::string& msg, const std::string& delim) {
    boost::split(tokens, msg, boost::is_any_of(delim));
}

int IsNmeaMessage(const char* buf, const int size) {
    // Start of sentence
    if (buf[0] != NMEA_PREAMBLE) {
        return 0;
    }

    // Find end of sentence, calculate checksum along the way
    int len = 1;  // Length of sentence excl. "$"
    char ck = 0;  // checksum
    while (true) {
        if (len > LIB_PARSER_MAX_NMEA_SIZE) {
            return 0;
        }
        if (len >= size)  // len doesn't include '$'
        {
            return -1;
        }
        if ((buf[len] == '\r') || (buf[len] == '\n') || (buf[len] == '*')) {
            break;
        }
        if (                                           // ((buf[len] & 0x80) != 0) || // 7-bit only
            (buf[len] < 0x20) || (buf[len] > 0x7e) ||  // valid range
            (buf[len] == '$') || (buf[len] == '\\') || (buf[len] == '!') || (buf[len] == '~'))  // reserved
        {
            return 0;
        }
        ck ^= buf[len];
        len++;
    }

    // Not nough data for sentence end (star + checksum + \r\n)?
    if (size < (len + 1 + 2 + 2)) {
        return -1;
    }

    // Properly terminated sentence?
    if ((buf[len] == '*') && (buf[len + 3] == '\r') && (buf[len + 4] == '\n')) {
        char n1 = buf[len + 1];
        char n2 = buf[len + 2];
        char c1 = '0' + ((ck >> 4) & 0x0f);
        char c2 = '0' + (ck & 0x0f);
        if (c2 > '9') {
            c2 += 'A' - '9' - 1;
        }
        // Checksum valid?
        if ((n1 == c1) && (n2 == c2)) {
            return len + 5;
        }
    }
    return 0;
}
}  // namespace fixposition