/*!
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Copyright (c) Fixposition AG
 *   /  /\  \   All right reserved
 *  /__/  \__\
 *
 *  Portions copyright NovAtel:
 *
 *     Copyright (c) 2020 NovAtel Inc.
 *
 *     Permission is hereby granted, free of charge, to any person obtaining a copy
 *     of this software and associated documentation files (the "Software"), to deal
 *     in the Software without restriction, including without limitation the rights
 *     to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *     copies of the Software, and to permit persons to whom the Software is
 *     furnished to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included in all
 *     copies or substantial portions of the Software.
 *
 *     THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *     IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *     FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *     AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *     LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *     OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *     SOFTWARE.
 * \endverbatim
 *
 *   @file
 *   @brief NovAtel types and utilities
 */

/* ****************************************************************************************************************** */
#ifndef __FIXPOSITION_DRIVER_LIB_NOV_TYPE__
#define __FIXPOSITION_DRIVER_LIB_NOV_TYPE__

// Fundamental types
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ****************************************************************************************************************** */

namespace fixposition {

/**
 * @brief CRC32 calculation
 *
 * @param[in] data
 * @param[in] size
 * @return uint32_t
 */
inline uint32_t nov_crc32(const uint8_t* data, const int size) {
    uint32_t crc = 0;
    for (int i = 0; i < size; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xedb88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static constexpr uint8_t SYNC_CHAR_1 = 0xaa;
static constexpr uint8_t SYNC_CHAR_2 = 0x44;
static constexpr uint8_t SYNC_CHAR_3_LONG = 0x12;
static constexpr uint8_t SYNC_CHAR_3_SHORT = 0x13;

/* ****************************************************************************************************************** */
/**
 * @name Novatel oem7 message flag definitions
 * @{
 */
enum class MessageId : uint16_t {
    GPGGA = 218,
    RAWIMU = 268,
    INSPVA = 507,
    HEADING2 = 1335,
    BESTPOS = 42,
    BERSTXYZ = 241,
    BESTGNSSPOS = 1429,
    INSPVAX = 1465,
    BESTXYZ = 241,
    BESTUTM = 726,
    BESTVEL = 99,
    CORRIMUS = 2264,
    IMURATECORRIMUS = 1362,
    INSCONFIG = 1945,
    INSPVAS = 508,
    INSSTDEV = 2051,
    PSRDOP2 = 1163,
    RXSTATUS = 93,
    TIME = 101
};

/**
 * @brief See https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSATTX.htm#ExtendedSolutionStatus
 * Bitwise Masks for all 32 bits. For usage please refer to INSPVAX converter.
 */
enum class ExtendedSolutionStatusIns : uint32_t {
    POSITION_UPDATE = 0x00000001,
    PHASE_UPDATE = 0x00000002,
    ZERO_VEOLICYT_UPDATE = 0x00000004,
    WHEEL_SENSOR_UPDATE = 0x00000008,
    HEADING_UPDATE = 0x00000010,
    EXTERNAL_POSITION_UPDATE = 0x00000020,
    INS_SOLUTION_CONVERGENCE = 0x00000040,
    DOPPLER_UPDATE = 0x00000080,
    PSEUDORANGE_UPDATE = 0x00000100,
    VELOCITY_UPDATE = 0x00000200,
    // RESERVED = 0x00000400,
    DR_UPDATE = 0x000000800,
    PHASE_WINDUP_UPDATE = 0x00001000,
    COURSE_OVER_GROUND_UPDATE = 0x00002000,
    EXTERNAL_VELOCITY_UPDATE = 0x00004000,
    EXTERNAL_ATTITUDE_UPDATE = 0x00008000,

    EXTERNAL_HEADING_UPDATE = 0x00010000,
    EXTERNAL_HEIGHT_UPDATE = 0x00020000,
    // RESERVED = 0x00040000,
    // RESERVED = 0x00080000,

    // RESERVED = 0x00100000,
    // RESERVED = 0x00200000,
    // RESERVED = 0x00400000,
    // RESERVED = 0x00800000,

    TURN_ON_BIAS_ESTIMATED = 0x01000000,
    ALIGNMENT_DIRECTION_VERIFIED = 0x02000000,
    ALIGNMENT_INDICATION_1 = 0x04000000,
    ALIGNMENT_INDICATION_2 = 0x08000000,

    ALIGNMENT_INDICATION_3 = 0x10000000,
    NVM_SEED_INDICATION_1 = 0x20000000,
    NVM_SEED_INDICATION_2 = 0x40000000,
    NVM_SEED_INDICATION_3 = 0x80000000,
};

/**
 * @brief See https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#GPS_GLONASSSignalUsedMask
 */
enum class GpsGlonassSignalUsed : uint8_t {
    GPS_L1 = 0x01,
    GPS_L2 = 0x02,
    GPS_L5 = 0x04,
    GLONASS_L1 = 0x10,
    GLONASS_L2 = 0x20,
    GLONASS_L5 = 0x40,
};

/**
 * @brief See https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#Galileo_BeiDouSignalUsedMask
 */
enum class GalileoBeidouSignalUsed : uint8_t {
    GALILEO_L1 = 0x01,
    GALILEO_L2 = 0x02,
    GALILEO_L5 = 0x04,
    BEIDOU_L1 = 0x10,
    BEIDOU_L2 = 0x20,
    BEIDOU_L5 = 0x40,
};

/**
 * @brief See https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSATT.htm#InertialSolutionStatus
 */
enum class InertialSolutionStatus : uint32_t {
    INS_INACTIVE = 0,
    INS_ALIGNING = 1,
    INS_HIGH_VARIANCE = 2,
    INS_SOLUTION_GOOD = 3,
    INS_SOLUTION_FREE = 6,
    INS_ALIGNMENT_COMPLETE = 7,
    DETERMINING_ORIENTATION = 8,
    WAITING_INITIAL_POS = 9,
    WAITING_AZIMUTH = 10,
    INITIALIZING_BIASES = 11,
    MOTION_DETECT = 12
};

/**
 * @brief See https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#Position_VelocityType
 */
enum class PositionOrVelocityType : uint32_t {
    NONE = 0,
    FIXEDPOS = 1,
    FIXEDHEIGHT = 2,
    DOPPLER_VELOCITY = 8,
    SINGLE = 16,
    PSRDIFF = 17,
    WAAS = 18,
    PROPAGATED = 19,
    L1_FLOAT = 32,
    NARROW_FLOAT = 34,
    L1_INT = 48,
    WIDE_INT = 49,
    NARROW_INT = 50,
    RTK_DIRECT_INS = 51,
    INS_SBAS = 52,
    INS_PSRSP = 53,
    INS_PSRDIFF = 54,
    INS_RTKFLOAT = 55,
    INS_RTKFIXED = 56,
    PPP_CONVERGING = 68,
    PPP = 69,
    OPERATIONAL = 70,
    WARNING = 71,
    OUT_OF_BOUNDS = 72,
    INS_PPP_CONVERGING = 73,
    INS_PPP = 74,
    PPP_BASIC_CONVERGING = 77,
    PPP_BASIC = 78,
    INS_PPP_BASIC_CONVERGING = 79,
    INS_PPP_BASIC = 80
};

/**
 * @brief See https://docs.novatel.com/OEM7/Content/Messages/GPS_Reference_Time_Statu.htm#Table_GPSReferenceTimeStatus
 */
enum class GpsReferenceTimeStatus : uint8_t {
    UNKNOWN = 20,
    APPROXIMATE = 60,
    COARSEADJUSTING = 80,
    COARSE = 100,
    COARSESTEERING = 120,
    FREEWHEELING = 130,
    FINEADJUSTING = 140,
    FINE = 160,
    FINEBACKUPSTEERING = 170,
    FINESTEERING = 180,  // I think we mainly use this one
    SATTIME = 200
};

/**
 * @brief Stringify time status
 * @param[in]  time_status  The time status
 * @returns a unique string for the time status
 */
const char* GpsReferenceTimeStatusStr(const GpsReferenceTimeStatus time_status);

enum class MessageTypeSource : uint8_t {  // clang-format off
    PRIMARY   = 0b00000000, //!< Primary antenna
    SECONDARY = 0b00000001, //!< Secondary antenna
    _MASK     = 0b00011111, //!< Mask for the source part of the message_type field
};  // clang-format on

/**
 * @brief See
 * https://docs.novatel.com/OEM7/Content/Messages/Binary.htm?tocpath=Commands%20%2526%20Logs%7CMessages%7C_____3#Table_BinaryMessageHeaderStructure
 */
enum class MessageTypeFormat : uint8_t {  // clang-format off
    BINARY      = 0b00000000, //!< Binary
    ASCII       = 0b00100000, //!< ASCII
    AASCII_NMEA = 0b01000000, //!< Abbreviated ASCII, NMEA
    RESERVED    = 0b01100000, //!< Reserved
    _MASK       = 0b01100000, //!< Mask for the format part of the message_type field
};  // clang-format on

enum class MessageTypeResponse : uint8_t {  // clang-format off
    ORIGINAL = 0b00000000,
    RESPONSE = 0b10000000,
    _MASK    = 0b10000000, //!< Mask for the response part of the message_type field
};  // clang-format on

enum class PortAddress : uint8_t {  // clang-format off
    NO_PORTS  = 0x00, //!< No ports specified
    ALL_PORTS = 0x80, //!< All virtual ports for all ports
    THISPORT  = 0xc0, //!< Current COM port
    // there are many more...
};  // clang-format on

// https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#SolutionStatus
enum class SolStat : uint32_t {  // clang-format off

    SOL_COMPUTED     =  0, //!< Solution computed
    INSUFFICIENT_OBS =  1, //!< Insufficient observations
    NO_CONVERGENCE   =  2, //!< No convergence
    SINGULARITY      =  3, //!< Singularity at parameters matrix
    COV_TRACE        =  4, //!< Covariance trace exceeds maximum (trace > 1000 m)
    TEST_DIST        =  5, //!< Test distance exceeded (maximum of 3 rejections if distance >10 km)
    COLD_START       =  6, //!< Not yet converged from cold start
    V_H_LIMIT        =  7, //!< Height or velocity limits exceeded (in accordance with export licensing restrictions)
    VARIANCE         =  8, //!< Variance exceeds limits
    RESIDUALS        =  9, //!< Residuals are too large
    // there are some more...
};  // clang-format on

enum class DatumId : uint32_t {
    WGS84 = 61,
    USER = 63,
};

// https://docs.novatel.com/OEM7/Content/Logs/HEADING2.htm#SolutionSource
enum class SolutionSource : uint8_t {  // clang-format off
    PRIMARY   = 0b00000100, //!< Primary antenna
    SECONDARY = 0b00001000, //!< Secondary antenna
    _MASK     = 0b00001100, //!< Mask
};  // clang-format on

// https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#ExtendedSolutionStatus
enum class ExtendedSolutionStatusGnss : uint8_t {  // clang-format off
    SOL_VERIFIED = 0b00000001, //!< Solution verified
    // There are more...
};  // clang-format on

/**
 * @}
 */
/* ****************************************************************************************************************** */
/**
 * @name Binary format definitions for Oem7 messages
 *
 * All Oem7 messages are 4-byte aligned, allowing simple casting into structs
 * @{
 */

typedef uint32_t oem7_enum_t;
typedef uint32_t oem7_bool_t;
typedef uint8_t oem7_hex_t;
typedef char oem7_char_t;

static_assert(sizeof(oem7_char_t) == 1, "");
static_assert(sizeof(double) == 8, "");
static_assert(sizeof(float) == 4, "");

struct __attribute__((packed)) Oem7MessageCommonHeaderMem {
    char sync1;
    char sync2;
    char sync3;

    uint8_t message_length;
    uint16_t message_id;
};

struct __attribute__((packed)) Oem7MessageHeaderMem {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t sync3;

    uint8_t header_length;
    uint16_t message_id;
    uint8_t message_type;
    uint8_t port_address;
    uint16_t message_length;
    uint16_t sequence;
    uint8_t idle_time;
    uint8_t time_status;
    uint16_t gps_week;
    int32_t gps_milliseconds;
    uint32_t receiver_status;
    uint16_t reserved;
    uint16_t receiver_version;
};

struct __attribute__((packed)) Oem7MessgeShortHeaderMem {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t sync3;

    uint8_t message_length;
    uint16_t message_id;
    uint16_t gps_week;
    int32_t gps_milliseconds;
};

struct __attribute__((packed)) BESTPOSMem {
    oem7_enum_t sol_stat;
    oem7_enum_t pos_type;
    double lat;
    double lon;
    double hgt;
    float undulation;
    oem7_enum_t datum_id;
    float lat_stdev;
    float lon_stdev;
    float hgt_stdev;
    oem7_char_t stn_id[4];
    float diff_age;
    float sol_age;
    uint8_t num_svs;
    uint8_t num_sol_svs;
    uint8_t num_sol_l1_svs;
    uint8_t num_sol_multi_svs;
    oem7_hex_t reserved;
    oem7_hex_t ext_sol_stat;
    uint8_t galileo_beidou_sig_mask;
    uint8_t gps_glonass_sig_mask;
};
static_assert(sizeof(BESTPOSMem) == 72, "");
// Could be changed: include more fields to become exactly same as pos, include short header or long while reading
struct __attribute__((packed)) BESTXYZMem {
    oem7_enum_t p_sol_stat;
    oem7_enum_t pos_type;
    double p_x;
    double p_y;
    double p_z;
    float p_x_stdev;
    float p_y_stdev;
    float p_z_stdev;
    oem7_enum_t v_sol_stat;
    oem7_enum_t vel_type;
    double v_x;
    double v_y;
    double v_z;
    float v_x_stdev;
    float v_y_stdev;
    float v_z_stdev;
};
static_assert(sizeof(BESTXYZMem) == 88, "");

struct __attribute__((packed)) BESTVELMem {
    uint32_t sol_stat;
    uint32_t vel_type;
    float latency;
    float diff_age;
    double hor_speed;
    double track_gnd;
    double ver_speed;
    float reserved;
};
static_assert(sizeof(BESTVELMem) == 44, "");

struct __attribute__((packed)) INSPVASmem {
    uint32_t gnss_week;
    double seconds;
    double latitude;
    double longitude;
    double height;
    double north_velocity;
    double east_velocity;
    double up_velocity;
    double roll;
    double pitch;
    double azimuth;
    oem7_enum_t status;
};
static_assert(sizeof(INSPVASmem) == 88, "");

struct __attribute__((packed)) CORRIMUSMem {
    uint32_t imu_data_count;
    double pitch_rate;
    double roll_rate;
    double yaw_rate;
    double lateral_acc;
    double longitudinal_acc;
    double vertical_acc;
    uint32_t reserved1;
    uint32_t reserved2;
};
static_assert(sizeof(CORRIMUSMem) == 60, "");

struct __attribute__((packed)) IMURATECORRIMUSMem {
    uint32_t week;
    double seconds;
    double pitch_rate;
    double roll_rate;
    double yaw_rate;
    double lateral_acc;
    double longitudinal_acc;
    double vertical_acc;
};
static_assert(sizeof(IMURATECORRIMUSMem) == 60, "");

struct __attribute__((packed)) INSSTDEVMem {
    float latitude_stdev;
    float longitude_stdev;
    float height_stdev;
    float north_velocity_stdev;
    float east_velocity_stdev;
    float up_velocity_stdev;
    float roll_stdev;
    float pitch_stdev;
    float azimuth_stdev;
    uint32_t ext_sol_status;
    uint16_t time_since_last_update;
    uint16_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
};
static_assert(sizeof(INSSTDEVMem) == 52, "");

struct __attribute__((packed)) INSCONFIG_FixedMem {
    oem7_enum_t imu_type;
    uint8_t mapping;
    uint8_t initial_alignment_velocity;
    uint16_t heave_window;
    oem7_enum_t profile;
    oem7_hex_t enabled_updates[4];
    oem7_enum_t alignment_mode;
    oem7_enum_t relative_ins_output_frame;
    oem7_bool_t relative_ins_output_direction;
    oem7_hex_t ins_receiver_status[4];
    uint8_t ins_seed_enabled;
    uint8_t ins_seed_validation;
    uint16_t reserved_1;
    uint32_t reserved_2;
    uint32_t reserved_3;
    uint32_t reserved_4;
    uint32_t reserved_5;
    uint32_t reserved_6;
    uint32_t reserved_7;
};
static_assert(sizeof(INSCONFIG_FixedMem) == 60, "");

struct __attribute__((packed)) INSCONFIG_TranslationMem {
    uint32_t translation;
    uint32_t frame;
    float x_offset;
    float y_offset;
    float z_offset;
    float x_uncertainty;
    float y_uncertainty;
    float z_uncertainty;
    uint32_t translation_source;
};

struct __attribute__((packed)) INSCONFIG_RotationMem {
    uint32_t rotation;
    uint32_t frame;
    float x_rotation;
    float y_rotation;
    float z_rotation;
    float x_rotation_stdev;
    float y_rotation_stdev;
    float z_rotation_stdev;
    uint32_t rotation_source;
};

struct __attribute__((packed)) INSPVAXMem {
    oem7_enum_t ins_status;
    oem7_enum_t pos_type;
    double latitude;
    double longitude;
    double height;
    float undulation;
    double north_velocity;
    double east_velocity;
    double up_velocity;
    double roll;
    double pitch;
    double azimuth;
    float latitude_stdev;
    float longitude_stdev;
    float height_stdev;
    float north_velocity_stdev;
    float east_velocity_stdev;
    float up_velocity_stdev;
    float roll_stdev;
    float pitch_stdev;
    float azimuth_stdev;
    uint32_t extended_status;
    uint16_t time_since_update;
};
static_assert(sizeof(INSPVAXMem) == 126, "");

struct __attribute__((packed)) HEADING2Mem {
    oem7_enum_t sol_status;
    oem7_enum_t pos_type;
    float length;
    float heading;
    float pitch;
    float reserved;
    float heading_stdev;
    float pitch_stdev;
    oem7_char_t rover_stn_id[4];
    oem7_char_t master_stn_id[4];
    uint8_t num_sv_tracked;
    uint8_t num_sv_in_sol;
    uint8_t num_sv_obs;
    uint8_t num_sv_multi;
    uint8_t sol_source;
    uint8_t ext_sol_status;
    uint8_t galileo_beidou_sig_mask;
    uint8_t gps_glonass_sig_mask;
};
static_assert(sizeof(HEADING2Mem) == 48, "");

struct __attribute__((packed)) BESTUTMMem {
    oem7_enum_t sol_stat;
    oem7_enum_t pos_type;
    uint32_t lon_zone_number;
    uint32_t lat_zone_letter;
    double northing;
    double easting;
    double height;
    float undulation;
    uint32_t datum_id;
    float northing_stddev;
    float easting_stddev;
    float height_stddev;
    char stn_id[4];
    float diff_age;
    float sol_age;
    uint8_t num_svs;
    uint8_t num_sol_svs;
    uint8_t num_sol_ggl1_svs;
    uint8_t num_sol_multi_svs;
    uint8_t reserved;
    uint8_t ext_sol_stat;
    uint8_t galileo_beidou_sig_mask;
    uint8_t gps_glonass_sig_mask;
};
static_assert(sizeof(BESTUTMMem) == 80, "");

struct __attribute__((packed)) RXSTATUSMem {
    uint32_t error;
    uint32_t num_status_codes;
    uint32_t rxstat;
    uint32_t rxstat_pri_mask;
    uint32_t rxstat_set_mask;
    uint32_t rxstat_clr_mask;
    uint32_t aux1_stat;
    uint32_t aux1_stat_pri;
    uint32_t aux1_stat_set;
    uint32_t aux1_stat_clr;
    uint32_t aux2_stat;
    uint32_t aux2_stat_pri;
    uint32_t aux2_stat_set;
    uint32_t aux2_stat_clr;
    uint32_t aux3_stat;
    uint32_t aux3_stat_pri;
    uint32_t aux3_stat_set;
    uint32_t aux3_stat_clr;
    uint32_t aux4_stat;
    uint32_t aux4_stat_pri;
    uint32_t aux4_stat_set;
    uint32_t aux4_stat_clr;
};
static_assert(sizeof(RXSTATUSMem) == 88, "");

struct __attribute__((packed)) TIMEMem {
    uint32_t clock_status;
    double offset;
    double offset_std;
    double utc_offset;
    uint32_t utc_year;
    uint8_t utc_month;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_min;
    uint32_t utc_msec;
    uint32_t utc_status;
};
static_assert(sizeof(TIMEMem) == 44, "");

struct __attribute__((packed)) PSRDOP2_FixedMem {
    float gdop;
    float pdop;
    float hdop;
    float vdop;
};
static_assert(sizeof(PSRDOP2_FixedMem) == 16, "");

struct __attribute__((packed)) PSRDOP2_SystemMem {
    uint32_t system;
    float tdop;
};

struct __attribute__((packed)) RAWIMUMem {
    uint32_t week;
    double seconds;
    uint32_t imu_stat;
    int32_t z_accel;
    int32_t y_accel;
    int32_t x_accel;
    int32_t z_gyro;
    int32_t y_gyro;
    int32_t x_gyro;
};
static_assert(sizeof(RAWIMUMem) == 40, "");

/**
 * @brief See https://docs.novatel.com/OEM7/Content/SPAN_Logs/BESTGNSSPOS.htm
 */
struct __attribute__((packed)) BESTGNSSPOSMem {
    oem7_enum_t sol_stat;
    oem7_enum_t pos_type;
    double lat;
    double lon;
    double hgt;
    float undulation;
    oem7_enum_t datum_id;
    float lat_stdev;
    float lon_stdev;
    float hgt_stdev;
    oem7_char_t stn_id[4];
    float diff_age;
    float sol_age;
    uint8_t num_svs;
    uint8_t num_sol_svs;
    uint8_t num_sol_l1_svs;
    uint8_t num_sol_multi_svs;
    oem7_hex_t reserved;
    oem7_hex_t ext_sol_stat;
    uint8_t galileo_beidou_sig_mask;
    uint8_t gps_glonass_sig_mask;
};
static_assert(sizeof(BESTGNSSPOSMem) == 72, "");

const size_t OEM7_BINARY_MSG_HDR_LEN = sizeof(Oem7MessageHeaderMem);
const size_t OEM7_BINARY_MSG_SHORT_HDR_LEN = sizeof(Oem7MessgeShortHeaderMem);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __LIB_NOVATEL_TYPE_H__
