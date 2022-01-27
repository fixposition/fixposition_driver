/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /
 *   /  /\  \
 *  /__/  \__\  Fixposition AG
 *
 * @file fixposition_driver.hpp
 * @author Kailin Huang (kailin.huang@fixposition.com)
 * @brief
 * @date 2022-01-26
 *
 */

#ifndef __FIXPOSITION_DRIVER_HELPER_HPP__
#define __FIXPOSITION_DRIVER_HELPER_HPP__
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
void split_message(std::vector<std::string>& tokens, const std::string& msg, const std::string& delim);

static constexpr const char NMEA_PREAMBLE = '$';
static constexpr const int LIB_PARSER_MAX_NMEA_SIZE = 400;

/**
 * @brief Check If msg is NMEA
 *
 * @param[in] buf start pointer of a char* buffer
 * @param[in] size size of the buffer to check
 * @return int the length of the NMEA message found. If no NMEA found then 0. If size argument is too small then -1
 */
int IsNmeaMessage(const char* buf, const int size);

}  // namespace fixposition
#endif