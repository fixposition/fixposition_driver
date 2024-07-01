/**
 *  @file
 *  @brief Helper functions
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

#ifndef __FIXPOSITION_DRIVER_LIB_HELPER__
#define __FIXPOSITION_DRIVER_LIB_HELPER__

/* SYSTEM / STL */
#include <string>
#include <vector>

/* EXTERNAL */
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fixposition_driver_lib/messages/msg_data.hpp>
#include <fixposition_driver_lib/messages/nov_type.hpp>

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
 * @brief
 *
 * @param[in] header
 * @param[in] bestgnsspos
 * @param[out] navsatfix
 */
void BestGnssPosToNavSatFix(const Oem7MessageHeaderMem* const header, const BESTGNSSPOSMem* const bestgnsspos,
                            NavSatFixData& navsatfix);

/**
 * @brief Convert NOV to other data structs
 *
 * @tparam T_nov NOV struct type
 * @tparam T_data Data struct type
 * @param[in] header
 * @param[in] nov
 * @param[out] data
 */
template <class T_nov, class T_data>
void NovToData(const Oem7MessageHeaderMem* const header, const T_nov* const nov, T_data& data);

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_HELPER__
