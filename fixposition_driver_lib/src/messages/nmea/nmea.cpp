/**
 *  @file
 *  @brief Implementation of NMEA-GX-GSV parser
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

/* PACKAGE */
#include <fixposition_driver_lib/messages/nmea_type.hpp>
#include <fixposition_driver_lib/messages/base_converter.hpp>

namespace fixposition {

void NmeaMessage::AddNmeaEpoch(const GP_GGA& msg) {
    // Populate time if empty
    if (time_str == "") {
        time_str = msg.time_str;
    }
    gpgga_time_str = msg.time_str;
    
    // Populate LLH position
    llh = msg.llh;

    // Populate quality
    quality = msg.quality;

    // Populate total number of satellites
    num_sv = msg.num_sv;

    // Populate GNSS receiver HDOP
    hdop_receiver = msg.hdop;

    // Populate differential data information
    diff_age = msg.diff_age;
    diff_sta = msg.diff_sta;
    
    // Populate position covariance [m^2]
    cov(0, 0) = msg.hdop * msg.hdop;
    cov(1, 1) = msg.hdop * msg.hdop;
    cov(2, 2) = 4 * msg.hdop * msg.hdop;
    cov(0, 1) = cov(1, 0) = 0.0;
    cov(0, 2) = cov(2, 0) = 0.0;
    cov(1, 2) = cov(2, 1) = 0.0;
}

void NmeaMessage::AddNmeaEpoch(const GP_GLL& msg) {
    // Populate time if empty
    if (time_str == "") {
        time_str = msg.time_str;
    }
    
    // Populate latitude and longitude if the vector is empty
    if (llh.isZero()) {
        llh(0) = msg.latlon(0);
        llh(1) = msg.latlon(1);
    }
}

void NmeaMessage::AddNmeaEpoch(const GN_GSA& msg) {
    // Populate DOP values (priority)
    pdop = msg.pdop;
    hdop = msg.hdop;
    vdop = msg.vdop;

    // Populate satellite information (priority)
    ids = msg.ids;
}

void NmeaMessage::AddNmeaEpoch(const GP_GST& msg) {
    // Populate time if empty
    if (time_str == "") {
        time_str = msg.time_str;
    }

    // Populate GNSS pseudorange error statistics (priority)
    rms_range = msg.rms_range;
    std_major = msg.std_major;
    std_minor = msg.std_minor;
    angle_major = msg.angle_major;
    std_lat = msg.std_lat;
    std_lon = msg.std_lon;
    std_alt = msg.std_alt;
}

void NmeaMessage::AddNmeaEpoch(const GX_GSV& msg) {
    // Get signal ID
    std::string signal_id;
    if (msg.constellation.size() >= 2) {
        signal_id = msg.constellation.substr(0,2) + msg.signal_id;
    } else {
        signal_id = msg.constellation + msg.signal_id;
    }
    
    // Create new struct if signal ID is not in map
    if (gnss_signals.find(signal_id) == gnss_signals.end()) {
        gnss_signals[signal_id] = GnssSignalStats();
    }

    // Get signal ID
    if (signal_id_lut.find(signal_id) == signal_id_lut.end()) {
        gnss_signals.at(signal_id).sat_id_name = "Key not found";
    } else {
        gnss_signals.at(signal_id).sat_id_name = signal_id_lut.at(signal_id);
    }

    // Get number of satellites in view
    gnss_signals.at(signal_id).num_sats = msg.num_sats;

    // Safety measure for vectors
    if (gnss_signals.at(signal_id).sat_id.size() >= 100) { gnss_signals.at(signal_id).sat_id.clear(); }
    if (gnss_signals.at(signal_id).azim.size() >= 100) { gnss_signals.at(signal_id).azim.clear(); }
    if (gnss_signals.at(signal_id).elev.size() >= 100) { gnss_signals.at(signal_id).elev.clear(); }
    if (gnss_signals.at(signal_id).cno.size() >= 100) { gnss_signals.at(signal_id).cno.clear(); }
    
    // Populate GNSS satellites in view (priority)
    gnss_signals.at(signal_id).sat_id.insert(gnss_signals.at(signal_id).sat_id.end(), msg.sat_id.begin(), msg.sat_id.end());
    gnss_signals.at(signal_id).azim.insert(gnss_signals.at(signal_id).azim.end(), msg.azim.begin(), msg.azim.end());
    gnss_signals.at(signal_id).elev.insert(gnss_signals.at(signal_id).elev.end(), msg.elev.begin(), msg.elev.end());
    gnss_signals.at(signal_id).cno.insert(gnss_signals.at(signal_id).cno.end(), msg.cno.begin(), msg.cno.end());
}

void NmeaMessage::AddNmeaEpoch(const GP_HDT& msg) {
    // Populate true heading (priority)
    heading = msg.heading;
}

void NmeaMessage::AddNmeaEpoch(const GP_VTG& msg) {
    // Populate SOG and COG if empty
    if (speed == 0.0) {
        speed = msg.sog_kph / 3.6; // Convert km/h to m/s
    }

    if (course == 0.0) {
        course = msg.cog_true;
    }
}

void NmeaMessage::AddNmeaEpoch(const GP_RMC& msg) {
    // Populate time if empty
    if (time_str == "") {
        time_str = msg.time_str;
    }

    // Populate time if empty
    if (date_str == "") {
        date_str = msg.date_str;
    }

    // Populate SOG and COG (priority)
    speed = msg.speed_ms;
    course = msg.course;
    
    // Populate latitude and longitude if the vector is empty
    if (llh.isZero()) {
        llh(0) = msg.latlon(0);
        llh(1) = msg.latlon(1);
    }
}

void NmeaMessage::AddNmeaEpoch(const GP_ZDA& msg) {
    // Populate time and date
    time_str = msg.time_str;
    date_str = msg.date_str;
    gpzda_time_str = msg.time_str;
}

}  // namespace fixposition
