/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Helper functions
 */

#ifndef __FIXPOSITION_DRIVER_LIB_HELPER_HPP__
#define __FIXPOSITION_DRIVER_LIB_HELPER_HPP__

/* LIBC/STL */
#include <string>
#include <vector>

/* EXTERNAL */
#include <fpsdk_common/parser/fpb.hpp>
#include <fpsdk_common/parser/novb.hpp>

/* PACKAGE */
#include "messages/msg_data.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief
 *
 * @param[in] header
 * @param[in] bestgnsspos
 * @param[out] navsatfix
 */
void BestGnssPosToNavSatFix(const fpsdk::common::parser::novb::NovbLongHeader& header,
                            const fpsdk::common::parser::novb::NovbBestgnsspos& bestgnsspos, NavSatFixData& navsatfix);

/**
 * @brief
 *
 * @param[in] meas_vec measurements from one specific wheelspeed sensor, with their validity flag
 * @param[in] meas_loc location from the specific wheelspeed sensor
 * @param[out] meas_fpb fpb measurement to be filled from the vector
 * @return true if the measurement was successfully filled, false otherwise
 */
bool FillWsSensorMeas(const std::vector<std::pair<bool, int>>& meas_vec,
                      const fpsdk::common::parser::fpb::FpbMeasurementsMeasLoc meas_loc,
                      fpsdk::common::parser::fpb::FpbMeasurementsMeas& meas_fpb);

/**
 * @brief Converts the measurement location from string to the enum values
 *
 * @param[in] meas_loc user input location in string format
 * @return FpbMeasurementsMeasLoc converted measurement location
 */
fpsdk::common::parser::fpb::FpbMeasurementsMeasLoc WsMeasStringToLoc(const std::string& meas_loc);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_HELPER_HPP__
