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
 * @brief Helpers to convert data to ROS 1 and 2 msgs
 */

#ifndef __FIXPOSITION_DRIVER_MSGS_DATA_TO_ROS_HPP__
#define __FIXPOSITION_DRIVER_MSGS_DATA_TO_ROS_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <fpsdk_common/ext/eigen_core.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/nmea.hpp>
#include <fpsdk_common/parser/novb.hpp>

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

// FpaFloat3 -> geometry_msgs::Vector3
template <typename RosMsgVector3T>
inline void FpaFloat3ToVector3(const fpsdk::common::parser::fpa::FpaFloat3 float3, RosMsgVector3T& vector3) {
    if (float3.valid) {
        vector3.x = float3.values[0];
        vector3.y = float3.values[1];
        vector3.z = float3.values[2];
    }
}

// ---------------------------------------------------------------------------------------------------------------------

// FpaFloat6 (covariance) -> builtin_types::float64[9]
template <typename RosMsgArray9T>
inline void FpaFloat6ToArray9(const fpsdk::common::parser::fpa::FpaFloat6 float6, RosMsgArray9T& array9) {
    if (float6.valid) {
        // clang-format off
        array9[0] = float6.values[0]; array9[1] = float6.values[3]; array9[2] = float6.values[5]; // 0 3 5   xx xy xz
        array9[3] = float6.values[3]; array9[4] = float6.values[1]; array9[5] = float6.values[4]; // 3 1 4   xy yy yz
        array9[6] = float6.values[5]; array9[7] = float6.values[4]; array9[8] = float6.values[2]; // 5 4 2   xz yz zz
        // clang-format on
    }
}

// ---------------------------------------------------------------------------------------------------------------------

template <typename RosMsgT>
inline int FpaGnssFixToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaGnssFix fix) {
    // clang-format off
    switch (fix) {
        case fpsdk::common::parser::fpa::FpaGnssFix::UNSPECIFIED: return msg.consts.GNSS_FIX_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaGnssFix::UNKNOWN:     return msg.consts.GNSS_FIX_UNKNOWN;
        case fpsdk::common::parser::fpa::FpaGnssFix::NOFIX:       return msg.consts.GNSS_FIX_NOFIX;
        case fpsdk::common::parser::fpa::FpaGnssFix::DRONLY:      return msg.consts.GNSS_FIX_DRONLY;
        case fpsdk::common::parser::fpa::FpaGnssFix::TIME:        return msg.consts.GNSS_FIX_TIME;
        case fpsdk::common::parser::fpa::FpaGnssFix::S2D:         return msg.consts.GNSS_FIX_S2D;
        case fpsdk::common::parser::fpa::FpaGnssFix::S3D:         return msg.consts.GNSS_FIX_S3D;
        case fpsdk::common::parser::fpa::FpaGnssFix::S3D_DR:      return msg.consts.GNSS_FIX_S3D_DR;
        case fpsdk::common::parser::fpa::FpaGnssFix::RTK_FLOAT:   return msg.consts.GNSS_FIX_RTK_FLOAT;
        case fpsdk::common::parser::fpa::FpaGnssFix::RTK_FIXED:   return msg.consts.GNSS_FIX_RTK_FIXED;
    }
    // clang-format on
    return msg.consts.GNSS_FIX_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaGnssFixToNavSatStatusStatus(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaGnssFix fix) {
    // clang-format off
    switch (fix) {
        case fpsdk::common::parser::fpa::FpaGnssFix::UNSPECIFIED: /* FALLTHROUGH */
        case fpsdk::common::parser::fpa::FpaGnssFix::UNKNOWN:     /* FALLTHROUGH */
        case fpsdk::common::parser::fpa::FpaGnssFix::NOFIX:       /* FALLTHROUGH */
        case fpsdk::common::parser::fpa::FpaGnssFix::DRONLY:      /* FALLTHROUGH */
        case fpsdk::common::parser::fpa::FpaGnssFix::TIME:        return msg.STATUS_NO_FIX;
        case fpsdk::common::parser::fpa::FpaGnssFix::S2D:         /* FALLTHROUGH */
        case fpsdk::common::parser::fpa::FpaGnssFix::S3D:         /* FALLTHROUGH */
        case fpsdk::common::parser::fpa::FpaGnssFix::S3D_DR:      return msg.STATUS_FIX;
        case fpsdk::common::parser::fpa::FpaGnssFix::RTK_FLOAT:   /* FALLTHROUGH */
        case fpsdk::common::parser::fpa::FpaGnssFix::RTK_FIXED:   return msg.STATUS_GBAS_FIX;
    }
    // clang-format on
    return msg.STATUS_NO_FIX;
}

template <typename RosMsgT>
inline int FpaFusionStatusLegacyToMsg(const RosMsgT& msg,
                                      const fpsdk::common::parser::fpa::FpaFusionStatusLegacy fusion_status) {
    // clang-format off
    switch (fusion_status) {
        case fpsdk::common::parser::fpa::FpaFusionStatusLegacy::UNSPECIFIED: return msg.consts.FUSION_STATUS_LEGACY_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaFusionStatusLegacy::NONE:        return msg.consts.FUSION_STATUS_LEGACY_NONE;
        case fpsdk::common::parser::fpa::FpaFusionStatusLegacy::VISION:      return msg.consts.FUSION_STATUS_LEGACY_VISION;
        case fpsdk::common::parser::fpa::FpaFusionStatusLegacy::VIO:         return msg.consts.FUSION_STATUS_LEGACY_VIO;
        case fpsdk::common::parser::fpa::FpaFusionStatusLegacy::IMU_GNSS:    return msg.consts.FUSION_STATUS_LEGACY_IMU_GNSS;
        case fpsdk::common::parser::fpa::FpaFusionStatusLegacy::VIO_GNSS:    return msg.consts.FUSION_STATUS_LEGACY_VIO_GNSS;
    }
    // clang-format on
    return msg.consts.FUSION_STATUS_LEGACY_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaImuStatusLegacyToMsg(const RosMsgT& msg,
                                   const fpsdk::common::parser::fpa::FpaImuStatusLegacy imu_bias_status) {
    // clang-format off
    switch (imu_bias_status) {
        case fpsdk::common::parser::fpa::FpaImuStatusLegacy::UNSPECIFIED:   return msg.consts.IMU_STATUS_LEGACY_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaImuStatusLegacy::NOT_CONVERGED: return msg.consts.IMU_STATUS_LEGACY_NOT_CONVERGED;
        case fpsdk::common::parser::fpa::FpaImuStatusLegacy::CONVERGED:     return msg.consts.IMU_STATUS_LEGACY_CONVERGED;
    }
    // clang-format on
    return msg.consts.IMU_STATUS_LEGACY_UNSPECIFIED;
    ;
}

template <typename RosMsgT>
inline int FpaWsStatusLegacyToMsg(const RosMsgT& msg,
                                  const fpsdk::common::parser::fpa::FpaWsStatusLegacy wheelspeed_status) {
    // clang-format off
    switch (wheelspeed_status) {
        case fpsdk::common::parser::fpa::FpaWsStatusLegacy::UNSPECIFIED:           return msg.consts.WS_STATUS_LEGACY_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaWsStatusLegacy::NOT_ENABLED:           return msg.consts.WS_STATUS_LEGACY_NOT_ENABLED;
        case fpsdk::common::parser::fpa::FpaWsStatusLegacy::NONE_CONVERGED:        return msg.consts.WS_STATUS_LEGACY_NONE_CONVERGED;
        case fpsdk::common::parser::fpa::FpaWsStatusLegacy::ONE_OR_MORE_CONVERGED: return msg.consts.WS_STATUS_LEGACY_ONE_OR_MORE_CONVERGED;
    }
    // clang-format on
    return msg.consts.WS_STATUS_LEGACY_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaInitStatusToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaInitStatus init_status) {
    // clang-format off
    switch (init_status) {
        case fpsdk::common::parser::fpa::FpaInitStatus::UNSPECIFIED: return msg.consts.INIT_STATUS_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaInitStatus::NOT_INIT:    return msg.consts.INIT_STATUS_NOT_INIT;
        case fpsdk::common::parser::fpa::FpaInitStatus::LOCAL_INIT:  return msg.consts.INIT_STATUS_LOCAL_INIT;
        case fpsdk::common::parser::fpa::FpaInitStatus::GLOBAL_INIT: return msg.consts.INIT_STATUS_GLOBAL_INIT;
    }
    // clang-format on
    return msg.consts.INIT_STATUS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaMeasStatusToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaMeasStatus status) {
    // clang-format off
    switch (status) {
        case fpsdk::common::parser::fpa::FpaMeasStatus::UNSPECIFIED: return msg.consts.MEAS_STATUS_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaMeasStatus::NOT_USED:    return msg.consts.MEAS_STATUS_NOT_USED;
        case fpsdk::common::parser::fpa::FpaMeasStatus::USED:        return msg.consts.MEAS_STATUS_USED;
        case fpsdk::common::parser::fpa::FpaMeasStatus::DEGRADED:    return msg.consts.MEAS_STATUS_DEGRADED;
    }
    // clang-format on
    return msg.consts.MEAS_STATUS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaImuStatusToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaImuStatus imu_status) {
    // clang-format off
    switch (imu_status) {
        case fpsdk::common::parser::fpa::FpaImuStatus::UNSPECIFIED:     return msg.consts.IMU_STATUS_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaImuStatus::NOT_CONVERGED:   return msg.consts.IMU_STATUS_NOT_CONVERGED;
        case fpsdk::common::parser::fpa::FpaImuStatus::WARMSTARTED:     return msg.consts.IMU_STATUS_WARMSTARTED;
        case fpsdk::common::parser::fpa::FpaImuStatus::ROUGH_CONVERGED: return msg.consts.IMU_STATUS_ROUGH_CONVERGED;
        case fpsdk::common::parser::fpa::FpaImuStatus::FINE_CONVERGED:  return msg.consts.IMU_STATUS_FINE_CONVERGED;
    }
    // clang-format on
    return msg.consts.IMU_STATUS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaImuNoiseToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaImuNoise imu_noise) {
    // clang-format off
    switch (imu_noise) {
        case fpsdk::common::parser::fpa::FpaImuNoise::UNSPECIFIED:  return msg.consts.IMU_NOISE_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaImuNoise::LOW_NOISE:    return msg.consts.IMU_NOISE_LOW_NOISE;
        case fpsdk::common::parser::fpa::FpaImuNoise::MEDIUM_NOISE: return msg.consts.IMU_NOISE_MEDIUM_NOISE;
        case fpsdk::common::parser::fpa::FpaImuNoise::HIGH_NOISE:   return msg.consts.IMU_NOISE_HIGH_NOISE;
        case fpsdk::common::parser::fpa::FpaImuNoise::RESERVED4:    return msg.consts.IMU_NOISE_RESERVED4;
        case fpsdk::common::parser::fpa::FpaImuNoise::RESERVED5:    return msg.consts.IMU_NOISE_RESERVED5;
        case fpsdk::common::parser::fpa::FpaImuNoise::RESERVED6:    return msg.consts.IMU_NOISE_RESERVED6;
        case fpsdk::common::parser::fpa::FpaImuNoise::RESERVED7:    return msg.consts.IMU_NOISE_RESERVED7;
    }
    // clang-format on
    return msg.consts.IMU_NOISE_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaImuConvToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaImuConv imu_conv) {
    // clang-format off
    switch (imu_conv) {
        case fpsdk::common::parser::fpa::FpaImuConv::UNSPECIFIED:      return msg.consts.IMU_CONV_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaImuConv::RESERVED0:        return msg.consts.IMU_CONV_RESERVED0;
        case fpsdk::common::parser::fpa::FpaImuConv::WAIT_IMU_MEAS:    return msg.consts.IMU_CONV_WAIT_IMU_MEAS;
        case fpsdk::common::parser::fpa::FpaImuConv::WAIT_GLOBAL_MEAS: return msg.consts.IMU_CONV_WAIT_GLOBAL_MEAS;
        case fpsdk::common::parser::fpa::FpaImuConv::WAIT_MOTION:      return msg.consts.IMU_CONV_WAIT_MOTION;
        case fpsdk::common::parser::fpa::FpaImuConv::CONVERGING:       return msg.consts.IMU_CONV_CONVERGING;
        case fpsdk::common::parser::fpa::FpaImuConv::RESERVED5:        return msg.consts.IMU_CONV_RESERVED5;
        case fpsdk::common::parser::fpa::FpaImuConv::RESERVED6:        return msg.consts.IMU_CONV_RESERVED6;
        case fpsdk::common::parser::fpa::FpaImuConv::IDLE:             return msg.consts.IMU_CONV_IDLE;
    }
    // clang-format on
    return msg.consts.IMU_CONV_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaGnssStatusToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaGnssStatus status) {
    // clang-format off
    switch (status) {
        case fpsdk::common::parser::fpa::FpaGnssStatus::UNSPECIFIED: return msg.consts.GNSS_STATUS_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaGnssStatus::NO_FIX:      return msg.consts.GNSS_STATUS_NO_FIX;
        case fpsdk::common::parser::fpa::FpaGnssStatus::SPP:         return msg.consts.GNSS_STATUS_SPP;
        case fpsdk::common::parser::fpa::FpaGnssStatus::RTK_MB:      return msg.consts.GNSS_STATUS_RTK_MB;
        case fpsdk::common::parser::fpa::FpaGnssStatus::RESERVED3:   return msg.consts.GNSS_STATUS_RESERVED3;
        case fpsdk::common::parser::fpa::FpaGnssStatus::RESERVED4:   return msg.consts.GNSS_STATUS_RESERVED4;
        case fpsdk::common::parser::fpa::FpaGnssStatus::RTK_FLOAT:   return msg.consts.GNSS_STATUS_RTK_FLOAT;
        case fpsdk::common::parser::fpa::FpaGnssStatus::RESERVED6:   return msg.consts.GNSS_STATUS_RESERVED6;
        case fpsdk::common::parser::fpa::FpaGnssStatus::RESERVED7:   return msg.consts.GNSS_STATUS_RESERVED7;
        case fpsdk::common::parser::fpa::FpaGnssStatus::RTK_FIXED:   return msg.consts.GNSS_STATUS_RTK_FIXED;
        case fpsdk::common::parser::fpa::FpaGnssStatus::RESERVED9:   return msg.consts.GNSS_STATUS_RESERVED9;
    }
    // clang-format on
    return msg.consts.GNSS_STATUS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaBaselineStatusToMsg(const RosMsgT& msg,
                                  const fpsdk::common::parser::fpa::FpaBaselineStatus baseline_status) {
    // clang-format off
    switch (baseline_status) {
        case fpsdk::common::parser::fpa::FpaBaselineStatus::UNSPECIFIED:    return msg.consts.BASELINE_STATUS_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaBaselineStatus::WAITING_FUSION: return msg.consts.BASELINE_STATUS_WAITING_FUSION;
        case fpsdk::common::parser::fpa::FpaBaselineStatus::NO_FIX:         return msg.consts.BASELINE_STATUS_NO_FIX;
        case fpsdk::common::parser::fpa::FpaBaselineStatus::FAILING:        return msg.consts.BASELINE_STATUS_FAILING;
        case fpsdk::common::parser::fpa::FpaBaselineStatus::PASSING:        return msg.consts.BASELINE_STATUS_PASSING;
    }
    // clang-format on
    return msg.consts.BASELINE_STATUS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaCorrStatusToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaCorrStatus corr_status) {
    // clang-format off
    switch (corr_status) {
        case fpsdk::common::parser::fpa::FpaCorrStatus::UNSPECIFIED:    return msg.consts.CORR_STATUS_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaCorrStatus::WAITING_FUSION: return msg.consts.CORR_STATUS_WAITING_FUSION;
        case fpsdk::common::parser::fpa::FpaCorrStatus::NO_GNSS:        return msg.consts.CORR_STATUS_NO_GNSS;
        case fpsdk::common::parser::fpa::FpaCorrStatus::NO_CORR:        return msg.consts.CORR_STATUS_NO_CORR;
        case fpsdk::common::parser::fpa::FpaCorrStatus::LIMITED_CORR:   return msg.consts.CORR_STATUS_LIMITED_CORR;
        case fpsdk::common::parser::fpa::FpaCorrStatus::OLD_CORR:       return msg.consts.CORR_STATUS_OLD_CORR;
        case fpsdk::common::parser::fpa::FpaCorrStatus::GOOD_CORR:      return msg.consts.CORR_STATUS_GOOD_CORR;
        case fpsdk::common::parser::fpa::FpaCorrStatus::RESERVED6:      return msg.consts.CORR_STATUS_RESERVED6;
        case fpsdk::common::parser::fpa::FpaCorrStatus::RESERVED7:      return msg.consts.CORR_STATUS_RESERVED7;
        case fpsdk::common::parser::fpa::FpaCorrStatus::RESERVED8:      return msg.consts.CORR_STATUS_RESERVED8;
        case fpsdk::common::parser::fpa::FpaCorrStatus::RESERVED9:      return msg.consts.CORR_STATUS_RESERVED9;
    }
    // clang-format on
    return msg.consts.CORR_STATUS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaCamStatusToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaCamStatus cam_status) {
    // clang-format off
    switch (cam_status) {
        case fpsdk::common::parser::fpa::FpaCamStatus::UNSPECIFIED: return msg.consts.CAM_STATUS_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaCamStatus::CAM_UNAVL:   return msg.consts.CAM_STATUS_CAM_UNAVL;
        case fpsdk::common::parser::fpa::FpaCamStatus::BAD_FEAT:    return msg.consts.CAM_STATUS_BAD_FEAT;
        case fpsdk::common::parser::fpa::FpaCamStatus::RESERVED2:   return msg.consts.CAM_STATUS_RESERVED2;
        case fpsdk::common::parser::fpa::FpaCamStatus::RESERVED3:   return msg.consts.CAM_STATUS_RESERVED3;
        case fpsdk::common::parser::fpa::FpaCamStatus::RESERVED4:   return msg.consts.CAM_STATUS_RESERVED4;
        case fpsdk::common::parser::fpa::FpaCamStatus::GOOD:        return msg.consts.CAM_STATUS_GOOD;
    }
    // clang-format on
    return msg.consts.CAM_STATUS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaWsStatusToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaWsStatus ws_status) {
    // clang-format off
    switch (ws_status) {
        case fpsdk::common::parser::fpa::FpaWsStatus::UNSPECIFIED:    return msg.consts.WS_STATUS_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaWsStatus::NOT_ENABLED:    return msg.consts.WS_STATUS_NOT_ENABLED;
        case fpsdk::common::parser::fpa::FpaWsStatus::MISS_MEAS:      return msg.consts.WS_STATUS_MISS_MEAS;
        case fpsdk::common::parser::fpa::FpaWsStatus::NONE_CONVERGED: return msg.consts.WS_STATUS_NONE_CONVERGED;
        case fpsdk::common::parser::fpa::FpaWsStatus::ONE_CONVERGED:  return msg.consts.WS_STATUS_ONE_CONVERGED;
        case fpsdk::common::parser::fpa::FpaWsStatus::ALL_CONVERGED:  return msg.consts.WS_STATUS_ALL_CONVERGED;
    }
    // clang-format on
    return msg.consts.WS_STATUS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaWsConvToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaWsConv ws_conv) {
    // clang-format off
    switch (ws_conv) {
        case fpsdk::common::parser::fpa::FpaWsConv::UNSPECIFIED:      return msg.consts.WS_CONV_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaWsConv::WAIT_FUSION:      return msg.consts.WS_CONV_WAIT_FUSION;
        case fpsdk::common::parser::fpa::FpaWsConv::WAIT_WS_MEAS:     return msg.consts.WS_CONV_WAIT_WS_MEAS;
        case fpsdk::common::parser::fpa::FpaWsConv::WAIT_GLOBAL_MEAS: return msg.consts.WS_CONV_WAIT_GLOBAL_MEAS;
        case fpsdk::common::parser::fpa::FpaWsConv::WAIT_MOTION:      return msg.consts.WS_CONV_WAIT_MOTION;
        case fpsdk::common::parser::fpa::FpaWsConv::WAIT_IMU_BIAS:    return msg.consts.WS_CONV_WAIT_IMU_BIAS;
        case fpsdk::common::parser::fpa::FpaWsConv::CONVERGING:       return msg.consts.WS_CONV_CONVERGING;
        case fpsdk::common::parser::fpa::FpaWsConv::IDLE:             return msg.consts.WS_CONV_IDLE;
    }
    // clang-format on
    return msg.consts.WS_CONV_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaMarkersStatusToMsg(const RosMsgT& msg,
                                 const fpsdk::common::parser::fpa::FpaMarkersStatus markers_status) {
    // clang-format off
    switch (markers_status) {
        case fpsdk::common::parser::fpa::FpaMarkersStatus::UNSPECIFIED:    return msg.consts.MARKERS_STATUS_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaMarkersStatus::NOT_ENABLED:    return msg.consts.MARKERS_STATUS_NOT_ENABLED;
        case fpsdk::common::parser::fpa::FpaMarkersStatus::NONE_CONVERGED: return msg.consts.MARKERS_STATUS_NONE_CONVERGED;
        case fpsdk::common::parser::fpa::FpaMarkersStatus::ONE_CONVERGED:  return msg.consts.MARKERS_STATUS_ONE_CONVERGED;
        case fpsdk::common::parser::fpa::FpaMarkersStatus::ALL_CONVERGED:  return msg.consts.MARKERS_STATUS_ALL_CONVERGED;
    }
    // clang-format on
    return msg.consts.MARKERS_STATUS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaMarkersConvToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaMarkersConv markers_conv) {
    // clang-format off
    switch (markers_conv) {
        case fpsdk::common::parser::fpa::FpaMarkersConv::UNSPECIFIED:      return msg.consts.MARKERS_CONV_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaMarkersConv::WAIT_FUSION:      return msg.consts.MARKERS_CONV_WAIT_FUSION;
        case fpsdk::common::parser::fpa::FpaMarkersConv::WAIT_MARKER_MEAS: return msg.consts.MARKERS_CONV_WAIT_MARKER_MEAS;
        case fpsdk::common::parser::fpa::FpaMarkersConv::WAIT_GLOBAL_MEAS: return msg.consts.MARKERS_CONV_WAIT_GLOBAL_MEAS;
        case fpsdk::common::parser::fpa::FpaMarkersConv::CONVERGING:       return msg.consts.MARKERS_CONV_CONVERGING;
        case fpsdk::common::parser::fpa::FpaMarkersConv::IDLE:             return msg.consts.MARKERS_CONV_IDLE;
    }
    // clang-format on
    return msg.consts.MARKERS_CONV_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaAntStateToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaAntState ant_state) {
    // clang-format off
    switch (ant_state) {
        case fpsdk::common::parser::fpa::FpaAntState::UNSPECIFIED: return msg.consts.ANT_STATE_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaAntState::OK:          return msg.consts.ANT_STATE_OK;
        case fpsdk::common::parser::fpa::FpaAntState::OPEN:        return msg.consts.ANT_STATE_OPEN;
        case fpsdk::common::parser::fpa::FpaAntState::SHORT:       return msg.consts.ANT_STATE_SHORT;
    }
    // clang-format on
    return msg.consts.ANT_STATE_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaAntPowerToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaAntPower ant_power) {
    // clang-format off
    switch (ant_power) {
        case fpsdk::common::parser::fpa::FpaAntPower::UNSPECIFIED: return msg.consts.ANT_POWER_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaAntPower::ON:          return msg.consts.ANT_POWER_ON;
        case fpsdk::common::parser::fpa::FpaAntPower::OFF:         return msg.consts.ANT_POWER_OFF;
    }
    // clang-format on
    return msg.consts.ANT_POWER_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaEpochToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaEpoch epoch) {
    // clang-format off
    switch (epoch)
    {
        case fpsdk::common::parser::fpa::FpaEpoch::UNSPECIFIED: return msg.consts.EPOCH_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaEpoch::GNSS1:       return msg.consts.EPOCH_GNSS1;
        case fpsdk::common::parser::fpa::FpaEpoch::GNSS2:       return msg.consts.EPOCH_GNSS2;
        case fpsdk::common::parser::fpa::FpaEpoch::GNSS:        return msg.consts.EPOCH_GNSS;
        case fpsdk::common::parser::fpa::FpaEpoch::FUSION:      return msg.consts.EPOCH_FUSION;
    }
    // clang-format on
    return msg.consts.EPOCH_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaTextLevelToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaTextLevel level) {
    // clang-format off
    switch (level)
    {
        case fpsdk::common::parser::fpa::FpaTextLevel::UNSPECIFIED: return msg.consts.TEXT_LEVEL_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaTextLevel::ERROR:       return msg.consts.TEXT_LEVEL_ERROR;
        case fpsdk::common::parser::fpa::FpaTextLevel::WARNING:     return msg.consts.TEXT_LEVEL_WARNING;
        case fpsdk::common::parser::fpa::FpaTextLevel::INFO:        return msg.consts.TEXT_LEVEL_INFO;
        case fpsdk::common::parser::fpa::FpaTextLevel::DEBUG:       return msg.consts.TEXT_LEVEL_DEBUG;
    }
    // clang-format on
    return msg.consts.TEXT_LEVEL_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaTimebaseToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaTimebase timebase) {
    // clang-format off
    switch (timebase)
    {
        case fpsdk::common::parser::fpa::FpaTimebase::UNSPECIFIED: return msg.consts.TIMEBASE_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaTimebase::NONE:        return msg.consts.TIMEBASE_NONE;
        case fpsdk::common::parser::fpa::FpaTimebase::GNSS:        return msg.consts.TIMEBASE_GNSS;
        case fpsdk::common::parser::fpa::FpaTimebase::UTC:         return msg.consts.TIMEBASE_UTC;
    }
    // clang-format on
    return msg.consts.TIMEBASE_UNSPECIFIED;
}

template <typename RosMsgT>
inline int FpaTimerefToMsg(const RosMsgT& msg, const fpsdk::common::parser::fpa::FpaTimeref timeref) {
    // clang-format off
    switch (timeref)
    {
        case fpsdk::common::parser::fpa::FpaTimeref::UNSPECIFIED: return msg.consts.TIMEREF_UNSPECIFIED;
        case fpsdk::common::parser::fpa::FpaTimeref::UTC_NONE:    return msg.consts.TIMEREF_UTC_NONE;
        case fpsdk::common::parser::fpa::FpaTimeref::UTC_CRL:     return msg.consts.TIMEREF_UTC_CRL;
        case fpsdk::common::parser::fpa::FpaTimeref::UTC_NIST:    return msg.consts.TIMEREF_UTC_NIST;
        case fpsdk::common::parser::fpa::FpaTimeref::UTC_USNO:    return msg.consts.TIMEREF_UTC_USNO;
        case fpsdk::common::parser::fpa::FpaTimeref::UTC_BIPM:    return msg.consts.TIMEREF_UTC_BIPM;
        case fpsdk::common::parser::fpa::FpaTimeref::UTC_EU:      return msg.consts.TIMEREF_UTC_EU;
        case fpsdk::common::parser::fpa::FpaTimeref::UTC_SU:      return msg.consts.TIMEREF_UTC_SU;
        case fpsdk::common::parser::fpa::FpaTimeref::UTC_NTSC:    return msg.consts.TIMEREF_UTC_NTSC;
        case fpsdk::common::parser::fpa::FpaTimeref::GNSS_GPS:    return msg.consts.TIMEREF_GNSS_GPS;
        case fpsdk::common::parser::fpa::FpaTimeref::GNSS_GAL:    return msg.consts.TIMEREF_GNSS_GAL;
        case fpsdk::common::parser::fpa::FpaTimeref::GNSS_BDS:    return msg.consts.TIMEREF_GNSS_BDS;
        case fpsdk::common::parser::fpa::FpaTimeref::GNSS_GLO:    return msg.consts.TIMEREF_GNSS_GLO;
        case fpsdk::common::parser::fpa::FpaTimeref::GNSS_NVC:    return msg.consts.TIMEREF_GNSS_NVC;
        case fpsdk::common::parser::fpa::FpaTimeref::OTHER:       return msg.consts.TIMEREF_OTHER;
    }
    // clang-format on
    return msg.consts.TIMEREF_UNSPECIFIED;
}

// ---------------------------------------------------------------------------------------------------------------------

template <typename RosMsgT>
inline int NovbPosOrVelTypeToNavSatStatusStatus(const RosMsgT& msg,
                                                const fpsdk::common::parser::novb::NovbPosOrVelType type) {
    switch (type) {
            // clang-format off
        case fpsdk::common::parser::novb::NovbPosOrVelType::NARROW_INT:   return msg.STATUS_GBAS_FIX; break;
        case fpsdk::common::parser::novb::NovbPosOrVelType::NARROW_FLOAT:
        case fpsdk::common::parser::novb::NovbPosOrVelType::SINGLE:       return msg.STATUS_FIX;      break;
        default:                                                          return msg.STATUS_NO_FIX;   break;
            // clang-format on
    }
}

// ---------------------------------------------------------------------------------------------------------------------

template <typename RosMsgT>
inline int NmeaTalkerIdToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaTalkerId talker) {
    // clang-format off
    switch (talker)
    {
        case fpsdk::common::parser::nmea::NmeaTalkerId::UNSPECIFIED: return msg.consts.TALKER_ID_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaTalkerId::PROPRIETARY: return msg.consts.TALKER_ID_PROPRIETARY;
        case fpsdk::common::parser::nmea::NmeaTalkerId::GPS_SBAS:    return msg.consts.TALKER_ID_GPS_SBAS;
        case fpsdk::common::parser::nmea::NmeaTalkerId::GLO:         return msg.consts.TALKER_ID_GLO;
        case fpsdk::common::parser::nmea::NmeaTalkerId::GAL:         return msg.consts.TALKER_ID_GAL;
        case fpsdk::common::parser::nmea::NmeaTalkerId::BDS:         return msg.consts.TALKER_ID_BDS;
        case fpsdk::common::parser::nmea::NmeaTalkerId::NAVIC:       return msg.consts.TALKER_ID_NAVIC;
        case fpsdk::common::parser::nmea::NmeaTalkerId::QZSS:        return msg.consts.TALKER_ID_QZSS;
        case fpsdk::common::parser::nmea::NmeaTalkerId::GNSS:        return msg.consts.TALKER_ID_GNSS;
    }
    // clang-format on
    return msg.consts.TALKER_ID_UNSPECIFIED;
}

template <typename RosMsgT>
inline int NmeaQualityGgaToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaQualityGga quality) {
    // clang-format off
    switch (quality)
    {
        case fpsdk::common::parser::nmea::NmeaQualityGga::UNSPECIFIED: return msg.consts.QUALITY_GGA_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaQualityGga::NOFIX:       return msg.consts.QUALITY_GGA_NOFIX;
        case fpsdk::common::parser::nmea::NmeaQualityGga::SPP:         return msg.consts.QUALITY_GGA_SPP;
        case fpsdk::common::parser::nmea::NmeaQualityGga::DGNSS:       return msg.consts.QUALITY_GGA_DGNSS;
        case fpsdk::common::parser::nmea::NmeaQualityGga::PPS:         return msg.consts.QUALITY_GGA_PPS;
        case fpsdk::common::parser::nmea::NmeaQualityGga::RTK_FIXED:   return msg.consts.QUALITY_GGA_RTK_FIXED;
        case fpsdk::common::parser::nmea::NmeaQualityGga::RTK_FLOAT:   return msg.consts.QUALITY_GGA_RTK_FLOAT;
        case fpsdk::common::parser::nmea::NmeaQualityGga::ESTIMATED:   return msg.consts.QUALITY_GGA_ESTIMATED;
        case fpsdk::common::parser::nmea::NmeaQualityGga::MANUAL:      return msg.consts.QUALITY_GGA_MANUAL;
        case fpsdk::common::parser::nmea::NmeaQualityGga::SIM:         return msg.consts.QUALITY_GGA_SIM;
    }
    // clang-format on
    return msg.consts.QUALITY_GGA_UNSPECIFIED;
}

template <typename RosMsgT>
inline int NmeaStatusGllRmcToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaStatusGllRmc status) {
    // clang-format off
    switch (status)
    {
        case fpsdk::common::parser::nmea::NmeaStatusGllRmc::UNSPECIFIED: return msg.consts.STATUS_GLL_RMC_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaStatusGllRmc::INVALID:     return msg.consts.STATUS_GLL_RMC_INVALID;
        case fpsdk::common::parser::nmea::NmeaStatusGllRmc::VALID:       return msg.consts.STATUS_GLL_RMC_VALID;
    }
    // clang-format on
    return msg.consts.STATUS_GLL_RMC_UNSPECIFIED;
}

template <typename RosMsgT>
inline int NmeaModeGllVtgToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaModeGllVtg mode) {
    // clang-format off
    switch (mode)
    {
        case fpsdk::common::parser::nmea::NmeaModeGllVtg::UNSPECIFIED: return msg.consts.MODE_GLL_VTG_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaModeGllVtg::INVALID:     return msg.consts.MODE_GLL_VTG_INVALID;
        case fpsdk::common::parser::nmea::NmeaModeGllVtg::AUTONOMOUS:  return msg.consts.MODE_GLL_VTG_AUTONOMOUS;
        case fpsdk::common::parser::nmea::NmeaModeGllVtg::DGNSS:       return msg.consts.MODE_GLL_VTG_DGNSS;
        case fpsdk::common::parser::nmea::NmeaModeGllVtg::ESTIMATED:   return msg.consts.MODE_GLL_VTG_ESTIMATED;
        case fpsdk::common::parser::nmea::NmeaModeGllVtg::MANUAL:      return msg.consts.MODE_GLL_VTG_MANUAL;
        case fpsdk::common::parser::nmea::NmeaModeGllVtg::SIM:         return msg.consts.MODE_GLL_VTG_SIM;
    }
    // clang-format on
    return msg.consts.MODE_GLL_VTG_UNSPECIFIED;
}

template <typename RosMsgT>
inline int NmeaModeRmcGnsToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaModeRmcGns mode) {
    // clang-format off
    switch (mode)
    {
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::UNSPECIFIED: return msg.consts.MODE_RMC_GNS_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::INVALID:     return msg.consts.MODE_RMC_GNS_INVALID;
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::AUTONOMOUS:  return msg.consts.MODE_RMC_GNS_AUTONOMOUS;
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::DGNSS:       return msg.consts.MODE_RMC_GNS_DGNSS;
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::ESTIMATED:   return msg.consts.MODE_RMC_GNS_ESTIMATED;
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::RTK_FIXED:   return msg.consts.MODE_RMC_GNS_RTK_FIXED;
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::RTK_FLOAT:   return msg.consts.MODE_RMC_GNS_RTK_FLOAT;
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::PRECISE:     return msg.consts.MODE_RMC_GNS_PRECISE;
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::MANUAL:      return msg.consts.MODE_RMC_GNS_MANUAL;
        case fpsdk::common::parser::nmea::NmeaModeRmcGns::SIM:         return msg.consts.MODE_RMC_GNS_SIM;
    }
    // clang-format on
    return msg.consts.MODE_RMC_GNS_UNSPECIFIED;
}

template <typename RosMsgT>
inline int NmeaNavStatusRmcToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaNavStatusRmc nav_status) {
    // clang-format off
    switch (nav_status)
    {
        case fpsdk::common::parser::nmea::NmeaNavStatusRmc::UNSPECIFIED: return msg.consts.NAV_STATUS_RMC_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaNavStatusRmc::NA:          return msg.consts.NAV_STATUS_RMC_NA;
        case fpsdk::common::parser::nmea::NmeaNavStatusRmc::SAFE:        return msg.consts.NAV_STATUS_RMC_SAFE;
        case fpsdk::common::parser::nmea::NmeaNavStatusRmc::CAUTION:     return msg.consts.NAV_STATUS_RMC_CAUTION;
        case fpsdk::common::parser::nmea::NmeaNavStatusRmc::UNSAFE:      return msg.consts.NAV_STATUS_RMC_UNSAFE;
    }
    // clang-format on
    return msg.consts.NAV_STATUS_RMC_UNSPECIFIED;
}

template <typename RosMsgT>
inline int NmeaOpModeGsaToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaOpModeGsa op_mode) {
    // clang-format off
    switch (op_mode)
    {
        case fpsdk::common::parser::nmea::NmeaOpModeGsa::UNSPECIFIED: return msg.consts.OP_MODE_GSA_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaOpModeGsa::MANUAL:      return msg.consts.OP_MODE_GSA_MANUAL;
        case fpsdk::common::parser::nmea::NmeaOpModeGsa::AUTO:        return msg.consts.OP_MODE_GSA_AUTO;
    }
    // clang-format on
    return msg.consts.OP_MODE_GSA_UNSPECIFIED;
}

template <typename RosMsgT>
inline int NmeaNavModeGsaToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaNavModeGsa nav_mode) {
    // clang-format off
    switch (nav_mode)
    {
        case fpsdk::common::parser::nmea::NmeaNavModeGsa::UNSPECIFIED: return msg.consts.NAV_MODE_GSA_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaNavModeGsa::NOFIX:       return msg.consts.NAV_MODE_GSA_NOFIX;
        case fpsdk::common::parser::nmea::NmeaNavModeGsa::FIX2D:       return msg.consts.NAV_MODE_GSA_FIX2D;
        case fpsdk::common::parser::nmea::NmeaNavModeGsa::FIX3D:       return msg.consts.NAV_MODE_GSA_FIX3D;
    }
    // clang-format on
    return msg.consts.NAV_MODE_GSA_UNSPECIFIED;
}

template <typename RosMsgT>
inline int NmeaSystemIdToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaSystemId system) {
    // clang-format off
    switch (system)
    {
        case fpsdk::common::parser::nmea::NmeaSystemId::UNSPECIFIED: return msg.consts.SYSTEM_ID_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaSystemId::GPS_SBAS:    return msg.consts.SYSTEM_ID_GPS_SBAS;
        case fpsdk::common::parser::nmea::NmeaSystemId::GLO:         return msg.consts.SYSTEM_ID_GLO;
        case fpsdk::common::parser::nmea::NmeaSystemId::GAL:         return msg.consts.SYSTEM_ID_GAL;
        case fpsdk::common::parser::nmea::NmeaSystemId::BDS:         return msg.consts.SYSTEM_ID_BDS;
        case fpsdk::common::parser::nmea::NmeaSystemId::QZSS:        return msg.consts.SYSTEM_ID_QZSS;
        case fpsdk::common::parser::nmea::NmeaSystemId::NAVIC:       return msg.consts.SYSTEM_ID_NAVIC;
    }
    // clang-format on
    return msg.consts.SYSTEM_ID_UNSPECIFIED;
}

template <typename RosMsgT>
inline int NmeaSignalIdToMsg(const RosMsgT& msg, const fpsdk::common::parser::nmea::NmeaSignalId signal) {
    // clang-format off
    switch (signal)
    {
        case fpsdk::common::parser::nmea::NmeaSignalId::UNSPECIFIED: return msg.consts.SIGNAL_ID_UNSPECIFIED;
        case fpsdk::common::parser::nmea::NmeaSignalId::NONE:        return msg.consts.SIGNAL_ID_NONE;
        case fpsdk::common::parser::nmea::NmeaSignalId::GPS_L1CA:    return msg.consts.SIGNAL_ID_GPS_L1CA;
        case fpsdk::common::parser::nmea::NmeaSignalId::GPS_L2CL:    return msg.consts.SIGNAL_ID_GPS_L2CL;
        case fpsdk::common::parser::nmea::NmeaSignalId::GPS_L2CM:    return msg.consts.SIGNAL_ID_GPS_L2CM;
        case fpsdk::common::parser::nmea::NmeaSignalId::GPS_L5I:     return msg.consts.SIGNAL_ID_GPS_L5I;
        case fpsdk::common::parser::nmea::NmeaSignalId::GPS_L5Q:     return msg.consts.SIGNAL_ID_GPS_L5Q;
        case fpsdk::common::parser::nmea::NmeaSignalId::GAL_E1:      return msg.consts.SIGNAL_ID_GAL_E1;
        case fpsdk::common::parser::nmea::NmeaSignalId::GAL_E5A:     return msg.consts.SIGNAL_ID_GAL_E5A;
        case fpsdk::common::parser::nmea::NmeaSignalId::GAL_E5B:     return msg.consts.SIGNAL_ID_GAL_E5B;
        case fpsdk::common::parser::nmea::NmeaSignalId::BDS_B1ID:    return msg.consts.SIGNAL_ID_BDS_B1ID;
        case fpsdk::common::parser::nmea::NmeaSignalId::BDS_B2ID:    return msg.consts.SIGNAL_ID_BDS_B2ID;
        case fpsdk::common::parser::nmea::NmeaSignalId::BDS_B1C:     return msg.consts.SIGNAL_ID_BDS_B1C;
        case fpsdk::common::parser::nmea::NmeaSignalId::BDS_B2A:     return msg.consts.SIGNAL_ID_BDS_B2A;
        case fpsdk::common::parser::nmea::NmeaSignalId::QZSS_L1CA:   return msg.consts.SIGNAL_ID_QZSS_L1CA;
        case fpsdk::common::parser::nmea::NmeaSignalId::QZSS_L1S:    return msg.consts.SIGNAL_ID_QZSS_L1S;
        case fpsdk::common::parser::nmea::NmeaSignalId::QZSS_L2CM:   return msg.consts.SIGNAL_ID_QZSS_L2CM;
        case fpsdk::common::parser::nmea::NmeaSignalId::QZSS_L2CL:   return msg.consts.SIGNAL_ID_QZSS_L2CL;
        case fpsdk::common::parser::nmea::NmeaSignalId::QZSS_L5I:    return msg.consts.SIGNAL_ID_QZSS_L5I;
        case fpsdk::common::parser::nmea::NmeaSignalId::QZSS_L5Q:    return msg.consts.SIGNAL_ID_QZSS_L5Q;
        case fpsdk::common::parser::nmea::NmeaSignalId::GLO_L1OF:    return msg.consts.SIGNAL_ID_GLO_L1OF;
        case fpsdk::common::parser::nmea::NmeaSignalId::GLO_L2OF:    return msg.consts.SIGNAL_ID_GLO_L2OF;
        case fpsdk::common::parser::nmea::NmeaSignalId::NAVIC_L5A:   return msg.consts.SIGNAL_ID_NAVIC_L5A;
        default:                                                     return msg.consts.SIGNAL_ID_UNSPECIFIED;
    }
    // clang-format on
    return msg.consts.SIGNAL_ID_UNSPECIFIED;
}

// ---------------------------------------------------------------------------------------------------------------------

template <typename RosMsgT>
inline int ParserProtocolToMsg(const RosMsgT& msg, const fpsdk::common::parser::Protocol protocol) {
    // clang-format off
    switch (protocol)
    {
        case fpsdk::common::parser::Protocol::FP_A:   return msg.PROTOCOL_FP_A;
        case fpsdk::common::parser::Protocol::FP_B:   return msg.PROTOCOL_FP_B;
        case fpsdk::common::parser::Protocol::NMEA:   return msg.PROTOCOL_NMEA;
        case fpsdk::common::parser::Protocol::UBX:    return msg.PROTOCOL_UBX;
        case fpsdk::common::parser::Protocol::RTCM3:  return msg.PROTOCOL_RTCM3;
        case fpsdk::common::parser::Protocol::UNI_B:  return msg.PROTOCOL_UNI_B;
        case fpsdk::common::parser::Protocol::NOV_B:  return msg.PROTOCOL_NOV_B;
        case fpsdk::common::parser::Protocol::SPARTN: return msg.PROTOCOL_SPARTN;
        case fpsdk::common::parser::Protocol::OTHER:  return msg.PROTOCOL_OTHER;
    }
    // clang-format on
    return msg.PROTOCOL_UNSPECIFIED;
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_MSGS_DATA_TO_ROS_HPP__
