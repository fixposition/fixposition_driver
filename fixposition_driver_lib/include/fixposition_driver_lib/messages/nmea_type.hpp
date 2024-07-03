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
    std::string sentence;

    // Message structure
    static constexpr char frame_id[] = "LLH";
    static constexpr char child_frame_id[] = "FP_POI";
    static constexpr char header_[] = "GPGGA";
    static constexpr unsigned int kSize_ = 15;

    GP_GGA() {
        time_str = "";
        llh.setZero();
        lat_ns = ' ';
        lon_ew = ' ';
        alt_unit = ' ';
        quality = -1;
        num_sv = -1;
        hdop = -1.0;
        diff_age = -1.0;
        diff_sta = "";
        sentence = "";
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        time_str = "";
        llh.setZero();
        lat_ns = ' ';
        lon_ew = ' ';
        alt_unit = ' ';
        quality = -1;
        num_sv = -1;
        hdop = -1.0;
        diff_age = -1.0;
        diff_sta = "";
        sentence = "";
    }
};

// ------------ NMEA-GP-GLL ------------

struct GP_GLL {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    std::string time_str;
    Eigen::Vector2d latlon;
    char lat_ns;
    char lon_ew;
    char status;
    char mode;

    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPGLL";
    static constexpr unsigned int kSize_ = 8;

    GP_GLL() {
        time_str = "";
        latlon.setZero();
        lat_ns = ' ';
        lon_ew = ' ';
        status = ' ';
        mode = ' ';
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        time_str = "";
        latlon.setZero();
        lat_ns = ' ';
        lon_ew = ' ';
        status = ' ';
        mode = ' ';
    }
};

// ------------ NMEA-GN-GSA ------------

struct GN_GSA {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    char mode_op;
    int8_t mode_nav;
    std::vector<int> ids;
    float pdop;
    float hdop;
    float vdop;
    int8_t gnss_id;

    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GNGSA";
    static constexpr unsigned int kSize_ = 19;

    GN_GSA() {
        mode_op = ' ';
        mode_nav = -1;
        ids.clear();
        pdop = -1.0;
        hdop = -1.0;
        vdop = -1.0;
        gnss_id = -1;
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        mode_op = ' ';
        mode_nav = -1;
        ids.clear();
        pdop = -1.0;
        hdop = -1.0;
        vdop = -1.0;
        gnss_id = -1;
    }
};

// ------------ NMEA-GP-GST ------------

struct GP_GST {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    std::string time_str;
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
    static constexpr unsigned int kSize_ = 9;

    GP_GST() {
        time_str = "";
        rms_range = -1.0;
        std_major = -1.0;
        std_minor = -1.0;
        angle_major = -1.0;
        std_lat = -1.0;
        std_lon = -1.0;
        std_alt = -1.0;
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        time_str = "";
        rms_range = -1.0;
        std_major = -1.0;
        std_minor = -1.0;
        angle_major = -1.0;
        std_lat = -1.0;
        std_lon = -1.0;
        std_alt = -1.0;
    }
};

// ------------ NMEA-GX-GSV ------------

struct GX_GSV {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    uint8_t sentences;
    uint8_t sent_num;
    uint8_t num_sats;
    std::vector<unsigned int> sat_id;
    std::vector<unsigned int> elev;
    std::vector<unsigned int> azim;
    std::vector<unsigned int> cno;
    std::string signal_id;
    SignalType type;

    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GXGST";
    unsigned int kSize_ = 4;  // Maximum size: 4 + num_sats * 4

    SignalType string2enum(const std::string& name) {
        if (name == "GP") return SignalType::GPS;     // GPS
        if (name == "GA") return SignalType::Galileo; // Galileo
        if (name == "GB") return SignalType::BeiDou;  // BeiDou
        if (name == "GL") return SignalType::GLONASS; // GLONASS
        return SignalType::Invalid;
    }

    GX_GSV() {
        sentences = 0;
        sent_num = 0;
        num_sats = 0;
        sat_id.clear();
        elev.clear();
        azim.clear();
        cno.clear();
        type = SignalType::Invalid;
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        sentences = 0;
        sent_num = 0;
        num_sats = 0;
        sat_id.clear();
        elev.clear();
        azim.clear();
        cno.clear();
        type = SignalType::Invalid;
    }
};

// ------------ NMEA-GP-HDT ------------

struct GP_HDT {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    float heading;
    char true_ind;

    // Message structure
    const std::string frame_id = "LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "GPHDT";
    static constexpr unsigned int kSize_ = 3;

    GP_HDT() {
        heading = 0.0;
        true_ind = ' ';
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        heading = 0.0;
        true_ind = ' ';
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
    float speed_ms;
    float course;
    char mode;

    // Message structure
    static constexpr char frame_id[] = "LLH";
    static constexpr char child_frame_id[] = "FP_POI";
    static constexpr char header_[] = "GPRMC";
    static constexpr unsigned int kSize_ = 13;

    GP_RMC() {
        date_str = "";
        time_str = "";
        status = ' ';
        latlon.setZero();
        lat_ns = ' ';
        lon_ew = ' ';
        speed = 0.0;
        speed_ms = 0.0;
        course = 0.0;
        mode = ' ';
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        date_str = "";
        time_str = "";
        status = ' ';
        latlon.setZero();
        lat_ns = ' ';
        lon_ew = ' ';
        speed = 0.0;
        speed_ms = 0.0;
        course = 0.0;
        mode = ' ';
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
    static constexpr char frame_id[] = "LLH";
    static constexpr char child_frame_id[] = "FP_POI";
    static constexpr char header_[] = "GPVTG";
    static constexpr unsigned int kSize_ = 10;

    GP_VTG() {
        cog_true = 0.0;
        cog_ref_t = ' ';
        cog_mag = 0.0;
        cog_ref_m = ' ';
        sog_knot = 0.0;
        sog_unit_n = ' ';
        sog_kph = 0.0;
        sog_unit_k = ' ';
        mode = ' ';
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        cog_true = 0.0;
        cog_ref_t = ' ';
        cog_mag = 0.0;
        cog_ref_m = ' ';
        sog_knot = 0.0;
        sog_unit_n = ' ';
        sog_kph = 0.0;
        sog_unit_k = ' ';
        mode = ' ';
    }
};

// ------------ NMEA-GP-ZDA ------------

struct GP_ZDA {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    std::string date_str;  // Format: dd/mm/yyyy
    std::string time_str;  // Format: hhmmss.ss(ss)
    times::GpsTime stamp;
    uint8_t local_hr;
    uint8_t local_min;

    // Message structure
    static constexpr char frame_id[] = "LLH";
    static constexpr char child_frame_id[] = "FP_POI";
    static constexpr char header_[] = "GPZDA";
    static constexpr unsigned int kSize_ = 7;

    GP_ZDA() {
        stamp = fixposition::times::GpsTime();
        time_str = "";
        date_str = "";
        local_hr = 0;
        local_min = 0;
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        stamp = fixposition::times::GpsTime();
        time_str = "";
        date_str = "";
        local_hr = 0;
        local_min = 0;
    }
};

// ------------ NmeaMessage ------------
struct GnssSignalStats {
    unsigned int azim;
    unsigned int elev;
    unsigned int cno_l1;
    unsigned int cno_l2;

    GnssSignalStats() {
        azim = 0;
        elev = 0;
        cno_l1 = 0;
        cno_l2 = 0;
    }

    void ResetData() {
        azim = 0;
        elev = 0;
        cno_l1 = 0;
        cno_l2 = 0;
    }
};

struct NmeaMessage {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    std::string gpgga_time_str;      // GP_GGA
    std::string gpzda_time_str;      // GP_ZDA
    std::string time_str;            // GP_ZDA (alt. GP_GGA, GP_GST, GP_RMC)
    std::string date_str;            // GP_ZDA (alt. GP_RMC)
    times::GpsTime stamp;            // GP_ZDA
    Eigen::Vector3d llh;             // GP_GGA (alt. LL only for GP_GLL, GP_RMC)
    uint8_t quality;                 // GP_GGA (alt. GP_RMC, GP_VTG, or limited GP_GLL, GP_GSA)
    uint8_t num_sv;                  // GP_GGA
    std::vector<int> ids;            // GN_GSA
    float hdop_receiver;             // GP_GGA
    float pdop;                      // GN_GSA
    float hdop;                      // GN_GSA (alt. GP_GGA)
    float vdop;                      // GN_GSA
    float rms_range;                 // GP_GST
    float std_major;                 // GP_GST
    float std_minor;                 // GP_GST
    float angle_major;               // GP_GST
    float std_lat;                   // GP_GST (alt. GP_GGA)
    float std_lon;                   // GP_GST (alt. GP_GGA)
    float std_alt;                   // GP_GST (alt. GP_GGA)
    Eigen::Matrix<double, 3, 3> cov; // GP_GST (alt. GP_GGA)
    uint8_t cov_type;                // GP_GST (alt. GP_GGA)
    float heading;                   // GP_HDT
    float speed;                     // GP_RMC (alt. GP_VTG)
    float course;                    // GP_RMC (alt. GP_VTG)
    float diff_age;                  // GP_GGA
    std::string diff_sta;            // GP_GGA
    std::unordered_map<SignalType, std::map<unsigned int, GnssSignalStats>> gnss_signals; // GX_GSV

    /**
     * @brief Check if GNSS epoch is complete
     */
    bool checkEpoch() {
        if ((gpgga_time_str.compare(gpzda_time_str) == 0)) {
            return true;
        } else {
            return false;
        }
    }

    NmeaMessage() {
        gpgga_time_str = "";
        gpzda_time_str = "";
        time_str = "";
        date_str = "";
        stamp = fixposition::times::GpsTime();
        llh.setZero();
        quality = -1;
        diff_age = 0.0;
        diff_sta = "";
        ids.clear();
        hdop_receiver = 0.0;
        pdop = 0.0;
        hdop = 0.0;
        vdop = 0.0;
        rms_range = 0.0;
        std_major = 0.0;
        std_minor = 0.0;
        angle_major = 0.0;
        std_lat = 0.0;
        std_lon = 0.0;
        std_alt = 0.0;
        cov.setZero();
        cov_type = 0;
        num_sv = 0;
        heading = 0.0;
        speed = 0.0;
        course = 0.0;
    }

    void ResetData() {
        gpgga_time_str = "";
        gpzda_time_str = "";
        time_str = "";
        date_str = "";
        stamp = fixposition::times::GpsTime();
        llh.setZero();
        quality = -1;
        diff_age = 0.0;
        diff_sta = "";
        ids.clear();
        hdop_receiver = 0.0;
        pdop = 0.0;
        hdop = 0.0;
        vdop = 0.0;
        rms_range = 0.0;
        std_major = 0.0;
        std_minor = 0.0;
        angle_major = 0.0;
        std_lat = 0.0;
        std_lon = 0.0;
        std_alt = 0.0;
        cov.setZero();
        cov_type = 0;
        num_sv = 0;
        heading = 0.0;
        speed = 0.0;
        course = 0.0;
    }

    void AddNmeaEpoch(const GP_GGA& msg);
    void AddNmeaEpoch(const GP_GLL& msg);
    void AddNmeaEpoch(const GN_GSA& msg);
    void AddNmeaEpoch(const GP_GST& msg);
    void AddNmeaEpoch(const GX_GSV& msg);
    void AddNmeaEpoch(const GP_HDT& msg);
    void AddNmeaEpoch(const GP_RMC& msg);
    void AddNmeaEpoch(const GP_VTG& msg);
    void AddNmeaEpoch(const GP_ZDA& msg);
};

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_NMEA_TYPE__
