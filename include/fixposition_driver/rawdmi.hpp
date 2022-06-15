/**
 *  @file
 *  @brief Declaration of RAWDMI message struct
 * 
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * 
 */

#ifndef __FIXPOSITION_DRIVER_RAWDMI__
#define __FIXPOSITION_DRIVER_RAWDMI__

#include <stdint.h>

namespace fixposition {

/**
 * @brief CRC32 calculation
 * 
 * @param[in] data 
 * @param[in] size 
 * @return uint32_t 
 */
inline uint32_t crc32(const uint8_t *data, const int size) {
    uint32_t crc = 0;
    for (int i = 0; i < size; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xedb88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief RAWDMI message struct
 * 
 */
struct RAWDMI {
    uint8_t head1;
    uint8_t head2;
    uint8_t head3;
    uint8_t payloadLen;
    uint16_t msgId;
    uint16_t wno;
    int32_t tow;
    int32_t dmi1;
    int32_t dmi2;
    int32_t dmi3;
    int32_t dmi4;
    int32_t mask;
};

}  // namespace fixposition

#endif  // __FIXPOSITION_DRIVER_RAWDMI__