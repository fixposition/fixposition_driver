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

/* LIBC/STL */

/* EXTERNAL */
#include <fpsdk_common/parser/crc.hpp>
#include <fpsdk_common/types.hpp>

/* PACKAGE */
#include "fixposition_driver_lib/helper.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::common;

static constexpr char kNmeaPreamble = '$';
static constexpr int kLibParserMaxNmeaSize = 400;

void BestGnssPosToNavSatFix(const parser::novb::NovbLongHeader& header,
                            const parser::novb::NovbBestgnsspos& bestgnsspos, NavSatFixData& navsatfix) {
    // Header timestamp
    navsatfix.stamp.wno = header.gps_week;
    navsatfix.stamp.tow = header.gps_milliseconds * 1e-3;

    // Data
    navsatfix.latitude = bestgnsspos.lat;
    navsatfix.longitude = bestgnsspos.lon;
    navsatfix.altitude = bestgnsspos.hgt;

    Eigen::Array3d cov_diag(bestgnsspos.lat_stdev, bestgnsspos.lon_stdev, bestgnsspos.hgt_stdev);

    navsatfix.cov = (cov_diag * cov_diag).matrix().asDiagonal();
    navsatfix.position_covariance_type = 2;

    switch (static_cast<parser::novb::NovbPosOrVelType>(bestgnsspos.pos_type)) {
        case parser::novb::NovbPosOrVelType::NARROW_INT:
            navsatfix.status.status = static_cast<int8_t>(NavSatStatusData::Status::STATUS_GBAS_FIX);
            break;
        case parser::novb::NovbPosOrVelType::NARROW_FLOAT:
        case parser::novb::NovbPosOrVelType::SINGLE:
            navsatfix.status.status = static_cast<int8_t>(NavSatStatusData::Status::STATUS_FIX);
            break;
        default:
            navsatfix.status.status = static_cast<int8_t>(NavSatStatusData::Status::STATUS_NO_FIX);
    }

    // TODO hardcoded for now for all 4 systems
    navsatfix.status.service = 0b000000000000000;
    navsatfix.status.service |= 1;
    navsatfix.status.service |= 2;
    navsatfix.status.service |= 4;
    navsatfix.status.service |= 8;

    switch (static_cast<parser::novb::NovbMsgTypeSource>(header.message_type &
                                                         types::EnumToVal(parser::novb::NovbMsgTypeSource::_MASK))) {
        case parser::novb::NovbMsgTypeSource::PRIMARY:
            navsatfix.frame_id = "GNSS1";
            break;
        case parser::novb::NovbMsgTypeSource::SECONDARY:
            navsatfix.frame_id = "GNSS2";
            break;
        default:
            navsatfix.frame_id = "GNSS";
    }
}

bool FillWsSensorMeas(const std::vector<std::pair<bool, int>>& meas_vec,
                      const parser::fpb::FpbMeasurementsMeasLoc meas_loc, parser::fpb::FpbMeasurementsMeas& meas_fpb) {
    const size_t num_axis = meas_vec.size();
    if (num_axis != 3) {
        WARNING("Wheelspeed sensor has an invalid number of measurements");
        return false;
    }
    meas_fpb.meas_type = types::EnumToVal(parser::fpb::FpbMeasurementsMeasType::VELOCITY);
    meas_fpb.meas_loc = types::EnumToVal(meas_loc);
    // In the current setup, the sensor will handle the timestamping as time of arrival.
    meas_fpb.timestamp_type = types::EnumToVal(parser::fpb::FpbMeasurementsTimestampType::TIMEOFARRIVAL);
    meas_fpb.gps_wno = 0;
    meas_fpb.gps_tow = 0;
    meas_fpb.meas_x_valid = meas_vec[0].first;
    meas_fpb.meas_x = meas_vec[0].second;
    meas_fpb.meas_y_valid = meas_vec[1].first;
    meas_fpb.meas_y = meas_vec[1].second;
    meas_fpb.meas_z_valid = meas_vec[2].first;
    meas_fpb.meas_z = meas_vec[2].second;
    return true;
}

parser::fpb::FpbMeasurementsMeasLoc WsMeasStringToLoc(const std::string& meas_loc) {
    // clang-format off
    if      (meas_loc == "RC") { return parser::fpb::FpbMeasurementsMeasLoc::RC; }
    else if (meas_loc == "FR") { return parser::fpb::FpbMeasurementsMeasLoc::FR; }
    else if (meas_loc == "FL") { return parser::fpb::FpbMeasurementsMeasLoc::FL; }
    else if (meas_loc == "RR") { return parser::fpb::FpbMeasurementsMeasLoc::RR; }
    else if (meas_loc == "RL") { return parser::fpb::FpbMeasurementsMeasLoc::RL; }
    else                       { return parser::fpb::FpbMeasurementsMeasLoc::UNSPECIFIED; }
    // clang-format on
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
