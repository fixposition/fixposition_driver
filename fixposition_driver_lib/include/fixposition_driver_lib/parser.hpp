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

#ifndef __FIXPOSITION_DRIVER_LIB_PARSER__
#define __FIXPOSITION_DRIVER_LIB_PARSER__

/* SYSTEM / STL */
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE*/

namespace fixposition {

/**
 * @brief Check If msg is NMEA
 *
 * @param[in] buf start pointer of a char* buffer
 * @param[in] size size of the buffer to check
 * @return int the length of the NMEA message found. If no NMEA found then 0. If size argument is too small then -1
 */
int IsNmeaMessage(const char* buf, const int size);

/**
 * @brief Check If msg is NOV_B
 *
 * @param[in] buf buffer ptr
 * @param[in] size size
 * @return int the length of the NOV_B message found. If no NOV_B found then 0. If size argument is too small then -1
 */
int IsNovMessage(const uint8_t* buf, const int size);

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_HELPER__
