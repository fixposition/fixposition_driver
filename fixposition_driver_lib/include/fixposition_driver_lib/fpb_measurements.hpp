/**
 *  @file
 *  @brief Declaration of FpbMeasurementsMeas message struct
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

#ifndef __FIXPOSITION_DRIVER_LIB_FPBMEASUREMENTS__
#define __FIXPOSITION_DRIVER_LIB_FPBMEASUREMENTS__

/* EXTERNAL */
#include <stdint.h>

/* PACKAGE */
#include <fixposition_driver_lib/fpb.hpp>

namespace fixposition {

struct FpbMeasurementsHeader {
    uint8_t  version;       //!< Message version (= FpbMeasurementsVersion for the current version of this message)
    uint8_t  num_meas;      //!< Number of measurements in the body (1..FP_B_MEASUREMENTS_MAX_NUM_MEAS)
    uint8_t  reserved0[6];  //!< Reserved for future use. Set to 0.
};
static_assert(sizeof(FpbMeasurementsHeader) == 8, "");
const int FP_B_MEASUREMENTS_HEAD_SIZE = 8;

enum FpbMeasurementsMeasType {
    MEASTYPE_UNSPECIFIED = 0,  //!< Message type unspecified, measurement will not be processed
    MEASTYPE_VELOCITY    = 1,  //!< Velocity message, measurement will be interpreted as a wheel speed
};

enum FpbMeasurementsMeasLoc {
    MEASLOC_UNSPECIFIED = 0,  //!< Location of the sensor unspecified, measurement will not be processed
    MEASLOC_RC          = 1,  //!< Measurement coming from the RC sensor
    MEASLOC_FR          = 2,  //!< Measurement coming from the FR sensor
    MEASLOC_FL          = 3,  //!< Measurement coming from the FL sensor
    MEASLOC_RR          = 4,  //!< Measurement coming from the RR sensor
    MEASLOC_RL          = 5,  //!< Measurement coming from the RL sensor
};

enum FpbMeasurementsTimestampType {
    TIME_UNSPECIFIED = 0,  //!< Timestamp type unspecified, measurement will not be processed
    TIME_TOA         = 1,  //!< Use time of arrival of the measurement
};

struct FpbMeasurementsMeas {
    int32_t  meas_x;          //!< Measurement x
    int32_t  meas_y;          //!< Measurement y
    int32_t  meas_z;          //!< Measurement z
    uint8_t  meas_x_valid;    //!< Validity of measurement x (1 = meas_x contains valid data, 0 = data invalid or n/a)
    uint8_t  meas_y_valid;    //!< Validity of measurement y (1 = meas_x contains valid data, 0 = data invalid or n/a)
    uint8_t  meas_z_valid;    //!< Validity of measurement z (1 = meas_x contains valid data, 0 = data invalid or n/a)
    uint8_t  meas_type;       //!< See FpbMeasurementsMeasType
    uint8_t  meas_loc;        //!< See FpbMeasurementsMeasLoc
    uint8_t  reserved1[4];    //!< Reserved for future use. Set to 0.
    uint8_t  timestamp_type;  //!< See FpbMeasurementsTimestampType
    uint16_t gps_wno;         //!< GPS week number [-]
    uint32_t gps_tow;         //!< GPS time of week [ms] or monotonic time [-]
};

static_assert(sizeof(FpbMeasurementsMeas) == 28, "");
const int FP_B_MEASUREMENTS_BODY_SIZE = 28;

}  // namespace fixposition

#endif  // __FIXPOSITION_DRIVER_LIB_FPBMEASUREMENTS__
