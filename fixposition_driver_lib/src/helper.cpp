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
 * @brief Helper functions and types
 */

/* LIBC/STL */
#include <utility>

/* EXTERNAL */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/crc.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/types.hpp>
#include <fpsdk_common/utils.hpp>

/* PACKAGE */
#include "fixposition_driver_lib/helper.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::common;
using namespace fpsdk::common::parser;

// ---------------------------------------------------------------------------------------------------------------------

time::Time FpaGpsTimeToTime(const fpa::FpaGpsTime gps_time) {
    time::Time time;
    // Check time is available and check that it is in range
    if (gps_time.week.valid && gps_time.tow.valid &&
        time.SetWnoTow({gps_time.week.value, gps_time.tow.value, time::WnoTow::Sys::GPS})) {
    }
    return time;
}

// ---------------------------------------------------------------------------------------------------------------------

bool PoseWithCovData::SetFromFpaOdomPayload(const fpa::FpaOdomPayload& payload) {
    bool ok = true;
    if (payload.pos.valid) {
        position = {payload.pos.values[0], payload.pos.values[1], payload.pos.values[2]};
    } else {
        ok = false;
    }
    if (payload.orientation.valid) {
        orientation = {payload.orientation.values[0], payload.orientation.values[1], payload.orientation.values[2],
                       payload.orientation.values[3]};
    } else {
        ok = false;
    }
    if (payload.pos_cov.valid && payload.orientation_cov.valid) {
        cov = BuildCovMat6D(payload.pos_cov.values[0], payload.pos_cov.values[1], payload.pos_cov.values[2],
                            payload.pos_cov.values[3], payload.pos_cov.values[4], payload.pos_cov.values[5],
                            payload.orientation_cov.values[0], payload.orientation_cov.values[1],
                            payload.orientation_cov.values[2], payload.orientation_cov.values[3],
                            payload.orientation_cov.values[4], payload.orientation_cov.values[5]);
        // DEBUG_S("PoseWithCovData.cov: " << cov);
    } else {
        ok = false;
    }

    valid = ok;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool TwistWithCovData::SetFromFpaOdomPayload(const fpa::FpaOdomPayload& payload) {
    bool ok = true;
    if (payload.vel.valid) {
        linear = {payload.vel.values[0], payload.vel.values[1], payload.vel.values[2]};
    } else {
        ok = false;
    }
    if (payload.rot.valid) {
        angular = {payload.rot.values[0], payload.rot.values[1], payload.rot.values[2]};
    } else {
        ok = false;
    }
    if (payload.vel_cov.valid) {
        cov = BuildCovMat6D(payload.vel_cov.values[0], payload.vel_cov.values[1], payload.vel_cov.values[2],
                            payload.vel_cov.values[3], payload.vel_cov.values[4], payload.vel_cov.values[5], 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0);
        // DEBUG_S("TwistWithCovData.cov: " << cov);

    } else {
        ok = false;
    }

    valid = ok;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool OdometryData::SetFromFpaOdomPayload(const fpa::FpaOdomPayload& payload) {
    bool ok = true;
    switch (payload.which) {
        case fpa::FpaOdomPayload::Which::ODOMETRY:
            frame_id = ODOMETRY_FRAME_ID;
            child_frame_id = ODOMETRY_CHILD_FRAME_ID;
            type = Type::ODOMETRY;
            break;
        case fpa::FpaOdomPayload::Which::ODOMSH:
            frame_id = ODOMSH_FRAME_ID;
            child_frame_id = ODOMSH_CHILD_FRAME_ID;
            type = Type::ODOMSH;
            break;
        case fpa::FpaOdomPayload::Which::ODOMENU:
            frame_id = ODOMENU_FRAME_ID;
            child_frame_id = ODOMENU_CHILD_FRAME_ID;
            type = Type::ODOMENU;
            break;
        case fpa::FpaOdomPayload::Which::UNSPECIFIED:
            ok = false;
            type = Type::UNSPECIFIED;
            break;
    }
    stamp = FpaGpsTimeToTime(payload.gps_time);
    if (stamp.IsZero()) {
        ok = false;
    }

    if (!pose.SetFromFpaOdomPayload(payload)) {
        ok = false;
    }
    if (!twist.SetFromFpaOdomPayload(payload)) {
        ok = false;
    }

    // Reset fields if orientation is bad FIXME: why?
    if ((std::fabs(pose.orientation.w()) < std::numeric_limits<double>::epsilon()) || pose.orientation.vec().isZero()) {
        pose = PoseWithCovData();
        twist = TwistWithCovData();
    }

    valid = ok;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool TfData::SetFromFpaTfPayload(const fpa::FpaTfPayload& payload) {
    bool ok = true;
    if (payload.gps_time.week.valid && payload.gps_time.tow.valid) {
        stamp.SetWnoTow({payload.gps_time.week.value, payload.gps_time.tow.value, time::WnoTow::Sys::GPS});
    } else {
        ok = false;
    }

    if (payload.frame_a[0] != '\0') {
        frame_id = "FP_" + std::string(payload.frame_a);
    } else {
        ok = false;
    }
    if (payload.frame_a[0] != '\0') {
        child_frame_id = "FP_" + std::string(payload.frame_b);
    } else {
        ok = false;
    }

    if (payload.translation.valid) {
        translation = {payload.translation.values[0], payload.translation.values[1], payload.translation.values[2]};
    } else {
        ok = false;
    }

    if (payload.orientation.valid) {
        rotation = {payload.orientation.values[0], payload.orientation.values[1], payload.orientation.values[2],
                    payload.orientation.values[3]};
    } else {
        ok = false;
    }

    // Check if TF is valid FIXME: Why?
    if ((std::fabs(rotation.w()) < std::numeric_limits<double>::epsilon()) || rotation.vec().isZero()) {
        ok = false;
    }

    valid = ok;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

JumpDetector::JumpDetector() {
    curr_pos_.setZero();
    curr_cov_.setZero();
    prev_pos_.setZero();
    prev_cov_.setZero();
    pos_diff_.setZero();
}

bool JumpDetector::Check(const OdometryData& odometry_data) {
    bool jump_detected = false;

    prev_pos_ = curr_pos_;
    prev_cov_ = curr_cov_;
    prev_stamp_ = curr_stamp_;

    if (!(prev_pos_.isZero() || prev_cov_.isZero())) {
        pos_diff_ = (prev_pos_ - odometry_data.pose.position).cwiseAbs();
        if ((pos_diff_[0] > prev_cov_(0, 0)) || (pos_diff_[1] > prev_cov_(1, 1)) || (pos_diff_[2] > prev_cov_(2, 2))) {
            jump_detected = true;
            warning_ = string::Sprintf(
                "Position jump detected! The change in position is greater than the estimated covariances. "
                "Position difference: [ %.4f, %.4f, %.4f ], Covariances: [ %.4f, %.4f, %.4f ]",
                pos_diff_[0], pos_diff_[1], pos_diff_[2], prev_cov_(0, 0), prev_cov_(1, 1), prev_cov_(2, 2));
        }
    }

    curr_pos_ = odometry_data.pose.position;
    curr_cov_ = odometry_data.pose.cov;
    curr_stamp_ = odometry_data.stamp;

    return jump_detected;
}

// ---------------------------------------------------------------------------------------------------------------------

NmeaEpochData::NmeaEpochData(const fpsdk::common::parser::fpa::FpaEpoch epoch) /* clang-format off */ :
    epoch_   { epoch }  // clang-format on
{
    cov_enu_.setZero();
}

// ---------------------------------------------------------------------------------------------------------------------

NmeaEpochData NmeaEpochData::CompleteAndReset() {
    // clang-format off
    switch (epoch_) {
        case fpa::FpaEpoch::UNSPECIFIED: break;
        case fpa::FpaEpoch::GNSS:        frame_id_ = GNSS_FRAME_ID; break;
        case fpa::FpaEpoch::GNSS1:       frame_id_ = GNSS1_FRAME_ID; break;
        case fpa::FpaEpoch::GNSS2:       frame_id_ = GNSS2_FRAME_ID; break;
        case fpa::FpaEpoch::FUSION:      frame_id_ = ODOMETRY_CHILD_FRAME_ID; break;
    }
    // clang-format on

    // Complete GSA and GSV collection
    gsa_gsv_.Complete();

    // Full date and time is available in RMC or ZDA
    if (rmc_.date.valid && rmc_.time.valid) {
        stamp_.SetUtcTime(
            {rmc_.date.years, rmc_.date.months, rmc_.date.days, rmc_.time.hours, rmc_.time.mins, rmc_.time.secs});
    } else if (zda_.date.valid && zda_.time.valid) {
        stamp_.SetUtcTime(
            {zda_.date.years, zda_.date.months, zda_.date.days, zda_.time.hours, zda_.time.mins, zda_.time.secs});
    }

    // Use best available info...
    // clang-format off
    if      (rmc_.date.valid) { date_ = rmc_.date; }
    else if (zda_.date.valid) { date_ = zda_.date; }
    if      (gga_.time.valid) { time_ = gga_.time; }
    else if (gll_.time.valid) { time_ = gll_.time; }
    else if (gst_.time.valid) { time_ = gst_.time; }
    else if (rmc_.time.valid) { time_ = rmc_.time; }
    else if (zda_.time.valid) { time_ = zda_.time; }
    if      (gga_.llh.latlon_valid) { llh_ = gga_.llh; }
    else if (rmc_.llh.latlon_valid) { llh_ = rmc_.llh; }
    else if (gll_.ll.latlon_valid)  { llh_ = gll_.ll; } // last as it does not have height
    // clang-format on

    status_ = (gll_.status > rmc_.status ? gll_.status : rmc_.status);
    navstatus_ = rmc_.navstatus;
    mode1_ = rmc_.mode;
    mode2_ = (gll_.mode > vtg_.mode ? gll_.mode : vtg_.mode);
    quality_ = gga_.quality;
    opmode_ = gsa_.opmode;
    navmode_ = gsa_.navmode;
    num_sv_ = gga_.num_sv;
    diff_age_ = gga_.diff_age;
    diff_sta_ = gga_.diff_sta;
    rms_range_ = gst_.rms_range;
    std_major_ = gst_.std_major;
    std_minor_ = gst_.std_minor;
    angle_major_ = gst_.angle_major;
    std_lat_ = gst_.std_lat;
    std_lon_ = gst_.std_lon;
    std_alt_ = gst_.std_alt;
    pdop_ = gsa_.pdop;
    hdop_ = (gsa_.hdop.valid ? gsa_.hdop : gga_.hdop);
    vdop_ = gsa_.vdop;
    heading_ = hdt_.heading;
    local_hr_ = zda_.local_hr;
    local_min_ = zda_.local_min;
    speed_ = rmc_.speed;
    course_ = rmc_.course;
    cogt_ = vtg_.cogt;
    cogm_ = vtg_.cogm;
    sogn_ = vtg_.sogn;
    sogk_ = vtg_.sogk;

    if (std_lat_.valid && std_lon_.valid) {
        cov_enu_(0, 0) = std_lon_.value * std_lon_.value;
        cov_enu_(1, 1) = std_lat_.value * std_lat_.value;
        cov_enu_(2, 2) = std_alt_.value * std_alt_.value;
    } else if (hdop_.valid && vdop_.valid) {
        cov_enu_(0, 0) = hdop_.value * hdop_.value;
        cov_enu_(1, 1) = hdop_.value * hdop_.value;
        cov_enu_(2, 2) = vdop_.value * vdop_.value;
    } else if (pdop_.valid) {
        cov_enu_(0, 0) = pdop_.value * pdop_.value;
        cov_enu_(1, 1) = pdop_.value * pdop_.value;
        cov_enu_(2, 2) = pdop_.value * pdop_.value * 4.0;
    }

    // Return data to user and reset ourselves back to empty
    NmeaEpochData data(epoch_);
    std::swap(data, *this);
    return data;
}

// ---------------------------------------------------------------------------------------------------------------------

void HelloWorld() {
    NOTICE("Fixposition driver (https://github.com/fixposition/fixposition_driver)");
    NOTICE("Fixposition SDK %s", utils::GetVersionString());
    NOTICE("%s", utils::GetCopyrightString());
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
