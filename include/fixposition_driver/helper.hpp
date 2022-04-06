/**
 *  @file
 *  @brief Helper functions
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_HELPER__
#define __FIXPOSITION_DRIVER_HELPER__

/* SYSTEM / STL */
#include <string>
#include <vector>

/* EXTERNAL */
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

namespace fixposition {

/**
 * @brief
 *
 * @param[in] msg
 * @param[in] delim
 * @param[out] tokens
 */
void SplitMessage(std::vector<std::string>& tokens, const std::string& msg, const std::string& delim);

/**
 * @brief Check If msg is NMEA
 *
 * @param[in] buf start pointer of a char* buffer
 * @param[in] size size of the buffer to check
 * @return int the length of the NMEA message found. If no NMEA found then 0. If size argument is too small then -1
 */
int IsNmeaMessage(const char* buf, const int size);

/**
 * @brief Send ROS Fatal error and exit
 *
 * @param[in] error Error msg to be sent
 */
void ROSFatalError(const std::string& error);

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_HELPER__