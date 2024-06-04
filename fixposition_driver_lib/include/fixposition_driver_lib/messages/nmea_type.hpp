/**
 *  @file
 *  @brief Declaration of NMEA type messages
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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_NMEA_TYPE__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_NMEA_TYPE__

/* SYSTEM / STL */

/* EXTERNAL */
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/* PACKAGE */
#include <fixposition_driver_lib/messages/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

// ------------ NMEA-GP-GGA ------------

struct GP_GGA {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    std::string time_str;
    Eigen::Vector3d llh;
    char lat_ns;
    char lon_ew;
    char alt_unit;
    int quality;
    int num_sv;
    float hdop;
    float diff_age;
    std::string diff_sta;
    
    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPGGA";
    static constexpr const int kSize_ = 15;

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        time_str = "";
        llh.setZero();
        lat_ns = '0';
        lon_ew = '0';
        alt_unit = '0';
        quality = 0;
        num_sv = 0;
        hdop = 0.0;
        diff_age = 0.0;
        diff_sta = ""; 
    }
};

// ------------ NMEA-GP-GLL ------------

struct GP_GLL {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    times::GpsTime stamp;
    Eigen::Vector2d latlon;
    char lat_ns;
    char lon_ew;
    char status;
    char mode;
    
    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPGLL";
    static constexpr const int kSize_ = 8;

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        stamp = fixposition::times::GpsTime();
        latlon.setZero();
        lat_ns = '0';
        lon_ew = '0';
        status = '0';
        mode = '0';
    }
};

// ------------ NMEA-GP-GSA ------------

struct GP_GSA {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    char mode_op;
    int8_t mode_nav;
    std::vector<int> ids[11];
    float pdop;
    float hdop;
    float vdop;
    int8_t gnss_id;
    
    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPGSA";
    static constexpr const int kSize_ = 19;

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        mode_op = '0';
        mode_nav = 0;
        memset(ids, 0, 11 * sizeof(ids[0]));
        pdop = 0.0;
        hdop = 0.0;
        vdop = 0.0;
        gnss_id = 0;
    }
};

// ------------ NMEA-GP-GST ------------

struct GP_GST {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    times::GpsTime stamp;
    float rms_range;
    float std_major;
    float std_minor;
    float angle_major;
    float std_lat;
    float std_lon;
    float std_alt;
    
    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPGST";
    static constexpr const int kSize_ = 9;

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        stamp = fixposition::times::GpsTime();
        rms_range = 0.0;
        std_major = 0.0;
        std_minor = 0.0;
        angle_major = 0.0;
        std_lat = 0.0;
        std_lon = 0.0;
        std_alt = 0.0;
    }
};

// ------------ NMEA-GP-GSV ------------

struct GP_GSV {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    uint8_t sentences;
    uint8_t sent_num;
    uint8_t num_sats;
    uint8_t sat_id[4];
    uint8_t elev[4];
    uint8_t azim[4];
    uint8_t cno[4];
    char signal_id;
    
    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPGST";
    int kSize_ = 4 + (4) * 4; // Maximum size: 4 + num_sats * 4

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        sentences = 0;
        sent_num = 0;
        num_sats = 0;
        memset(sat_id, 0, 4 * sizeof(sat_id[0]));
        memset(elev, 0, 4 * sizeof(elev[0]));
        memset(azim, 0, 4 * sizeof(azim[0]));
        memset(cno, 0, 4 * sizeof(cno[0]));
        signal_id = '0';
    }
};

// ------------ NMEA-GP-HDT ------------

struct GP_HDT {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    float heading;
    
    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPHDT";
    static constexpr const int kSize_ = 3;

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        heading = 0.0;
    }
};

// ------------ NMEA-GP-RMC ------------

struct GP_RMC {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    std::string date_str;
    std::string time_str;
    char status;
    Eigen::Vector2d latlon;
    char lat_ns;
    char lon_ew;
    float speed;
    float course;
    char mode;
    
    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPRMC";
    static constexpr const int kSize_ = 13;

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        time_str = "";
        status = '0';
        latlon.setZero();
        lat_ns = '0';
        lon_ew = '0';
        speed = 0.0;
        course = 0.0;
        mode = '0';
    }
};

// ------------ NMEA-GP-VTG ------------

struct GP_VTG {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    float cog_true;
    char cog_ref_t;
    float cog_mag;
    char cog_ref_m;
    float sog_knot;
    char sog_unit_n;
    float sog_kph;
    char sog_unit_k;
    char mode;
    
    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPVTG";
    static constexpr const int kSize_ = 10;

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        cog_true = 0.0;
        cog_ref_t = '0';
        cog_mag = 0.0;
        cog_ref_m = '0';
        sog_knot = 0.0;
        sog_unit_n = '0';
        sog_kph = 0.0;
        sog_unit_k = '0';
        mode = '0';
    }
};

// ------------ NMEA-GP-ZDA ------------

struct GP_ZDA {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    std::string date;
    std::string time;
    times::GpsTime stamp;
    uint8_t local_hr;
    uint8_t local_min;
    
    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPZDA";
    static constexpr const int kSize_ = 10;

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        stamp = fixposition::times::GpsTime();
        local_hr = 0;
        local_min = 0;
    }
};

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_NMEA_TYPE__
