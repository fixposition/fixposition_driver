/**
 *  @file
 *  @brief Declaration of RAWDMI message struct
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

#ifndef __FIXPOSITION_DRIVER_LIB_RAWDMI__
#define __FIXPOSITION_DRIVER_LIB_RAWDMI__

#include <stdint.h>
#include <fixposition_driver_lib/nov_type.hpp>

namespace fixposition {


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

#endif  // __FIXPOSITION_DRIVER_LIB_RAWDMI__
