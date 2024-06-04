/**
 *  @file
 *  @brief Parser functions
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
#include <fixposition_driver_lib/helper.hpp>
#include <fixposition_driver_lib/messages/nov_type.hpp>

namespace fixposition {

static constexpr const char kNmeaPreamble = '$';
static constexpr const int kLibParserMaxNmeaSize = 400;
static constexpr const int kLibParserMaxNovSize = 4096;

int IsNmeaMessage(const char* buf, const int size) {
    // Start of sentence
    if (buf[0] != kNmeaPreamble) {
        return 0;
    }

    // Find end of sentence, calculate checksum along the way
    int len = 1;  // Length of sentence excl. "$"
    char ck = 0;  // checksum
    while (true) {
        if (len > kLibParserMaxNmeaSize) {
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

int IsNovMessage(const uint8_t* buf, const int size) {
    if (buf[0] != SYNC_CHAR_1) {
        return 0;
    }

    if (size < 3) {
        return -1;
    }

    if ((buf[1] != SYNC_CHAR_2) || ((buf[2] != SYNC_CHAR_3_LONG) && (buf[2] != SYNC_CHAR_3_SHORT))) {
        return 0;
    }

    // https://docs.novatel.com/OEM7/Content/Messages/Binary.htm
    // https://docs.novatel.com/OEM7/Content/Messages/Description_of_Short_Headers.htm
    // Offset  Type      Long header   Short header
    // 0       uint8_t   0xaa         uint8_t   0xaa
    // 1       uint8_t   0x44         uint8_t   0x44
    // 2       uint8_t   0x12         uint8_t   0x13
    // 3       uint8_t   header len   uint8_t   msg len
    // 4       uint16_t  msg ID       uint16_t  msg ID
    // 6       uint8_t   msg type     uint16_t  wno
    // 7       uint8_t   port addr
    // 8       uint16_t  msg len      int32_t   tow
    // ...
    // 3+hlen  payload             offs 13 payload

    if (size < 12) {
        return -1;
    }

    int len = 0;

    // Long header
    if (buf[2] == SYNC_CHAR_3_LONG) {
        const uint8_t headerLen = buf[3];
        const uint16_t msgLen = ((uint16_t)buf[9] << 8) | (uint16_t)buf[8];
        len = headerLen + msgLen + sizeof(uint32_t);
    }
    // Short header
    else {
        const uint8_t headerLen = 12;
        const uint8_t msgLen = buf[3];
        len = headerLen + msgLen + sizeof(uint32_t);
    }

    if (len > kLibParserMaxNovSize) {
        return 0;
    }

    if (size < len) {
        return -1;
    }

    const uint32_t crc = (buf[len - 1] << 24) | (buf[len - 2] << 16) | (buf[len - 3] << 8) | (buf[len - 4]);
    if (crc == nov_crc32(buf, len - sizeof(uint32_t))) {
        return len;
    } else {
        return 0;
    }
}

}  // namespace fixposition
