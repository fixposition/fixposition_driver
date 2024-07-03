/**
 *  @file
 *  @brief Main function for the fixposition driver ros node
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
#include <fixposition_driver_ros1/fixposition_driver_node.hpp>

namespace fixposition {

FixpositionDriverNode::FixpositionDriverNode(const FixpositionDriverParams& params) : 
    FixpositionDriver(params), nh_("~"),
    // FP_A messages
    fpa_gnssant_pub_(nh_.advertise<fixposition_driver_ros1::gnssant>("/fixposition/fpa/gnssant", 10)),
    fpa_gnsscorr_pub_(nh_.advertise<fixposition_driver_ros1::gnsscorr>("/fixposition/fpa/gnsscorr", 10)),
    fpa_llh_pub_(nh_.advertise<fixposition_driver_ros1::llh>("/fixposition/fpa/llh", 10)),
    fpa_odomenu_pub_(nh_.advertise<fixposition_driver_ros1::odomenu>("/fixposition/fpa/odomenu", 10)),
    fpa_odometry_pub_(nh_.advertise<fixposition_driver_ros1::odometry>("/fixposition/fpa/odometry", 10)),
    fpa_odomsh_pub_(nh_.advertise<fixposition_driver_ros1::odomsh>("/fixposition/fpa/odomsh", 10)),
    fpa_text_pub_(nh_.advertise<fixposition_driver_ros1::text>("/fixposition/fpa/text", 10)),

    // NMEA messages
    nmea_gpgga_pub_(nh_.advertise<fixposition_driver_ros1::gpgga>("/fixposition/nmea/gpgga", 10)),
    nmea_gpgll_pub_(nh_.advertise<fixposition_driver_ros1::gpgll>("/fixposition/nmea/gpgll", 10)),
    nmea_gngsa_pub_(nh_.advertise<fixposition_driver_ros1::gngsa>("/fixposition/nmea/gngsa", 10)),
    nmea_gpgst_pub_(nh_.advertise<fixposition_driver_ros1::gpgst>("/fixposition/nmea/gpgst", 10)),
    nmea_gxgsv_pub_(nh_.advertise<fixposition_driver_ros1::gxgsv>("/fixposition/nmea/gxgsv", 100)),
    nmea_gphdt_pub_(nh_.advertise<fixposition_driver_ros1::gphdt>("/fixposition/nmea/gphdt", 10)),
    nmea_gprmc_pub_(nh_.advertise<fixposition_driver_ros1::gprmc>("/fixposition/nmea/gprmc", 10)),
    nmea_gpvtg_pub_(nh_.advertise<fixposition_driver_ros1::gpvtg>("/fixposition/nmea/gpvtg", 10)),
    nmea_gpzda_pub_(nh_.advertise<fixposition_driver_ros1::gpzda>("/fixposition/nmea/gpzda", 10)),

    // ODOMETRY
    odometry_ecef_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_ecef", 10)),
    odometry_llh_pub_(nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/odometry_llh", 10)),
    odometry_enu_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_enu", 10)),
    odometry_smooth_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_smooth", 10)),

    // Orientation
    eul_pub_(nh_.advertise<geometry_msgs::Vector3Stamped>("/fixposition/ypr", 10)),
    eul_imu_pub_(nh_.advertise<geometry_msgs::Vector3Stamped>("/fixposition/imu_ypr", 10)),

    // IMU
    rawimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/rawimu", 100)),
    corrimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/corrimu", 100)),
    poiimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/poiimu", 10)),

    // GNSS
    nmea_pub_(nh_.advertise<fixposition_driver_ros1::NMEA>("/fixposition/nmea", 10)),
    navsatfix_gnss1_pub_(nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/gnss1", 10)),
    navsatfix_gnss2_pub_(nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/gnss2", 10))

{
    ws_sub_ = nh_.subscribe<fixposition_driver_ros1::Speed>(params_.customer_input.speed_topic, 10,
                                                            &FixpositionDriverNode::WsCallback, this,
                                                            ros::TransportHints().tcpNoDelay());
    rtcm_sub_ = nh_.subscribe<std_msgs::UInt8MultiArray>(params_.customer_input.rtcm_topic, 10,
                                                         &FixpositionDriverNode::RtcmCallback, this,
                                                         ros::TransportHints().tcpNoDelay());
    RegisterObservers();
}

void FixpositionDriverNode::Run() {
    ros::Rate rate(params_.fp_output.rate);

    while (ros::ok()) {
        // Read data and publish to ros
        const bool connection_ok = RunOnce();
        // process Incoming ROS msgs
        ros::spinOnce();
        // Handle connection loss
        if (!connection_ok) {
            printf("Reconnecting in %.1f seconds ...\n", params_.fp_output.reconnect_delay);
            ros::Duration(params_.fp_output.reconnect_delay).sleep();
            Connect();
        } else {
            rate.sleep();  // ensure the loop runs at the desired rate
        }
    }
}

void FixpositionDriverNode::RegisterObservers() {
    // NOV_B
    bestgnsspos_obs_.push_back(std::bind(&FixpositionDriverNode::BestGnssPosToPublishNavSatFix, this,
                                         std::placeholders::_1, std::placeholders::_2));

    // FP_A
    for (const auto& format : params_.fp_output.formats) {
        if (format == "ODOMETRY") {
            dynamic_cast<NmeaConverter<FP_ODOMETRY>*>(a_converters_["ODOMETRY"].get())
                ->AddObserver([this](const FP_ODOMETRY& data) {
                    FpToRosMsg(data, fpa_odometry_pub_);
                    FpToRosMsg(data.odom, odometry_ecef_pub_);
                    OdomToImuMsg(data, poiimu_pub_);
                    OdomToNavSatFix(data, odometry_llh_pub_);
                    OdometryDataToTf(data, br_);
                });
        } else if (format == "ODOMENU") {
            dynamic_cast<NmeaConverter<FP_ODOMENU>*>(a_converters_["ODOMENU"].get())
                ->AddObserver([this](const FP_ODOMENU& data) {
                    FpToRosMsg(data, fpa_odomenu_pub_);
                    FpToRosMsg(data.odom, odometry_enu_pub_);
                    OdomToYprMsg(data.odom, eul_pub_);
                });
        } else if (format == "ODOMSH") {
            dynamic_cast<NmeaConverter<FP_ODOMSH>*>(a_converters_["ODOMSH"].get())
                ->AddObserver([this](const FP_ODOMSH& data) {
                    FpToRosMsg(data, fpa_odomsh_pub_);
                    FpToRosMsg(data.odom, odometry_smooth_pub_);
                });
        } else if (format == "LLH") {
            dynamic_cast<NmeaConverter<FP_LLH>*>(a_converters_["LLH"].get())
                ->AddObserver([this](const FP_LLH& data) { FpToRosMsg(data, fpa_llh_pub_); });
        } else if (format == "GNSSANT") {
            dynamic_cast<NmeaConverter<FP_GNSSANT>*>(a_converters_["GNSSANT"].get())
                ->AddObserver([this](const FP_GNSSANT& data) { FpToRosMsg(data, fpa_gnssant_pub_); });
        } else if (format == "GNSSCORR") {
            dynamic_cast<NmeaConverter<FP_GNSSCORR>*>(a_converters_["GNSSCORR"].get())
                ->AddObserver([this](const FP_GNSSCORR& data) { FpToRosMsg(data, fpa_gnsscorr_pub_); });
        } else if (format == "TEXT") {
            dynamic_cast<NmeaConverter<FP_TEXT>*>(a_converters_["TEXT"].get())
                ->AddObserver([this](const FP_TEXT& data) { FpToRosMsg(data, fpa_text_pub_); });
        } else if (format == "RAWIMU") {
            dynamic_cast<NmeaConverter<FP_RAWIMU>*>(a_converters_["RAWIMU"].get())
                ->AddObserver([this](const FP_RAWIMU& data) { FpToRosMsg(data.imu, rawimu_pub_); });
        } else if (format == "CORRIMU") {
            dynamic_cast<NmeaConverter<FP_CORRIMU>*>(a_converters_["CORRIMU"].get())
                ->AddObserver([this](const FP_CORRIMU& data) { FpToRosMsg(data.imu, corrimu_pub_); });
        } else if (format == "TF") {
            dynamic_cast<NmeaConverter<FP_TF>*>(a_converters_["TF"].get())->AddObserver([this](const FP_TF& data) {
                // TF Observer Lambda
                geometry_msgs::TransformStamped tf;
                TfDataToMsg(data.tf, tf);
                if (tf.child_frame_id == "FP_IMUH" && tf.header.frame_id == "FP_POI") {
                    br_.sendTransform(tf);

                    // Publish Pitch Roll based on IMU only
                    Eigen::Vector3d imu_ypr_eigen = gnss_tf::QuatToEul(data.tf.rotation);
                    imu_ypr_eigen.x() = 0.0;  // the yaw value is not observable using IMU alone
                    geometry_msgs::Vector3Stamped imu_ypr;
                    imu_ypr.header.stamp = tf.header.stamp;
                    imu_ypr.header.frame_id = "FP_POI";
                    tf::vectorEigenToMsg(imu_ypr_eigen, imu_ypr.vector);
                    eul_imu_pub_.publish(imu_ypr);

                } else if (tf.child_frame_id == "FP_POISH" && tf.header.frame_id == "FP_POI") {
                    br_.sendTransform(tf);
                } else {
                    static_br_.sendTransform(tf);
                }
            });
        } else if (format == "GPGGA") {
            dynamic_cast<NmeaConverter<GP_GGA>*>(a_converters_["GPGGA"].get())->AddObserver([this](const GP_GGA& data) {
                FpToRosMsg(data, nmea_gpgga_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) { 
                    nmea_message_.AddNmeaEpoch(data);
                    PublishNmea(); // GPGGA controls the NMEA output
                }
            });
        } else if (format == "GPGLL") {
            dynamic_cast<NmeaConverter<GP_GLL>*>(a_converters_["GPGLL"].get())->AddObserver([this](const GP_GLL& data) { 
                FpToRosMsg(data, nmea_gpgll_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) { nmea_message_.AddNmeaEpoch(data); }
            });
        } else if (format == "GNGSA") {
            dynamic_cast<NmeaConverter<GN_GSA>*>(a_converters_["GNGSA"].get())->AddObserver([this](const GN_GSA& data) { 
                FpToRosMsg(data, nmea_gngsa_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) { nmea_message_.AddNmeaEpoch(data); }
            });
        } else if (format == "GPGST") {
            dynamic_cast<NmeaConverter<GP_GST>*>(a_converters_["GPGST"].get())->AddObserver([this](const GP_GST& data) { 
                FpToRosMsg(data, nmea_gpgst_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) { nmea_message_.AddNmeaEpoch(data); }
            });
        } else if (format == "GXGSV") {
            dynamic_cast<NmeaConverter<GX_GSV>*>(a_converters_["GXGSV"].get())->AddObserver([this](const GX_GSV& data) { 
                FpToRosMsg(data, nmea_gxgsv_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) { nmea_message_.AddNmeaEpoch(data); }
            });
        } else if (format == "GPHDT") {
            dynamic_cast<NmeaConverter<GP_HDT>*>(a_converters_["GPHDT"].get())->AddObserver([this](const GP_HDT& data) { 
                FpToRosMsg(data, nmea_gphdt_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) { nmea_message_.AddNmeaEpoch(data); }
            });
        } else if (format == "GPRMC") {
            dynamic_cast<NmeaConverter<GP_RMC>*>(a_converters_["GPRMC"].get())->AddObserver([this](const GP_RMC& data) {
                FpToRosMsg(data, nmea_gprmc_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) { nmea_message_.AddNmeaEpoch(data); }
            });
        } else if (format == "GPVTG") {
            dynamic_cast<NmeaConverter<GP_VTG>*>(a_converters_["GPVTG"].get())->AddObserver([this](const GP_VTG& data) { 
                FpToRosMsg(data, nmea_gpvtg_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) { nmea_message_.AddNmeaEpoch(data); }
            });
        } else if (format == "GPZDA") {
            dynamic_cast<NmeaConverter<GP_ZDA>*>(a_converters_["GPZDA"].get())->AddObserver([this](const GP_ZDA& data) {
                FpToRosMsg(data, nmea_gpzda_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) { nmea_message_.AddNmeaEpoch(data); }
            });
        }
    }
}

void FixpositionDriverNode::PublishNmea() {
    // If epoch message is complete, generate NMEA output
    if (nmea_message_.checkEpoch()) {
        // Generate new message
        fixposition_driver_ros1::NMEA msg;

        // ROS Header
        if (nmea_message_.stamp.tow == 0.0 && nmea_message_.stamp.wno == 0) {
            msg.header.stamp = ros::Time::now();
        } else {
            msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(nmea_message_.stamp));
        }
        msg.header.frame_id = "FP_POI";

        // Time and date fields
        msg.time = nmea_message_.time_str;
        msg.date = nmea_message_.date_str;
        
        // Latitude [degrees]. Positive is north of equator; negative is south
        msg.latitude = nmea_message_.llh(0);

        // Longitude [degrees]. Positive is east of prime meridian; negative is west
        msg.longitude = nmea_message_.llh(1);

        // Altitude [m]. Positive is above the WGS-84 ellipsoid
        msg.altitude = nmea_message_.llh(2);

        // Quality indicator
        msg.quality = nmea_message_.quality;

        // Number of satellites
        msg.num_sv = nmea_message_.num_sv;

        // ID numbers of satellites used in solution
        for (unsigned int i = 0; i < nmea_message_.ids.size(); i++) {
           msg.ids.push_back(nmea_message_.ids.at(i));
        }

        // Dilution of precision
        msg.hdop_rec = nmea_message_.hdop_receiver;
        msg.pdop = nmea_message_.pdop;
        msg.hdop = nmea_message_.hdop;
        msg.vdop = nmea_message_.vdop;

        // Populate GNSS pseudorange error statistics
        msg.rms_range = nmea_message_.rms_range;
        msg.std_major = nmea_message_.std_major;
        msg.std_minor = nmea_message_.std_minor;
        msg.angle_major = nmea_message_.angle_major;
        msg.std_lat = nmea_message_.std_lat;
        msg.std_lon = nmea_message_.std_lon;
        msg.std_alt = nmea_message_.std_alt;

        // Position covariance [m^2]
        Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map =
            Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.covariance.data());
        cov_map = nmea_message_.cov;

        // Method employed to estimate covariance
        msg.cov_type = nmea_message_.cov_type;

        // Populate GNSS satellites in view
        for (auto it = nmea_message_.gnss_signals.begin(); it != nmea_message_.gnss_signals.end(); ++it) {
            GnssSignalStats gnss_data = it->second;

            // Populate GnssSats message
            fixposition_driver_ros1::GnssSats sats_msg;
            sats_msg.signal_id = gnss_data.sat_id_name;
            sats_msg.num_sats = gnss_data.num_sats;
            for (unsigned int i = 0; i < gnss_data.sat_id.size(); i++) {
                sats_msg.sat_id.push_back(gnss_data.sat_id.at(i));
                sats_msg.azim.push_back(gnss_data.azim.at(i));
                sats_msg.elev.push_back(gnss_data.elev.at(i));
                sats_msg.cno.push_back(gnss_data.cno.at(i));
            }
            msg.gnss_sats.push_back(sats_msg);
            
            // Clear vectors
            it->second.sat_id.clear();
            it->second.azim.clear();
            it->second.elev.clear();
            it->second.cno.clear();
        }

        // True heading
        msg.heading = nmea_message_.heading;

        // Speed over ground [m/s]
        msg.speed = nmea_message_.speed;

        // Course over ground [deg]
        msg.course = nmea_message_.course;

        // Populate differential data information
        msg.diff_age = nmea_message_.diff_age;
        msg.diff_sta = nmea_message_.diff_sta;

        // Publish message
        nmea_pub_.publish(msg);
    }
}

void FixpositionDriverNode::WsCallback(const fixposition_driver_ros1::SpeedConstPtr& msg) {
    std::unordered_map<std::string, std::vector<std::pair<bool, int>>> measurements;
    for (const auto &sensor : msg->sensors) {
        measurements[sensor.location].push_back({sensor.vx_valid, sensor.vx});
        measurements[sensor.location].push_back({sensor.vy_valid, sensor.vy});
        measurements[sensor.location].push_back({sensor.vz_valid, sensor.vz});
    }
    FixpositionDriver::WsCallback(measurements);
}

void FixpositionDriverNode::RtcmCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    const void *rtcm_msg = &(msg->data[0]);
    size_t msg_size = msg->layout.dim[0].size;
    FixpositionDriver::RtcmCallback(rtcm_msg, msg_size);
}

void FixpositionDriverNode::BestGnssPosToPublishNavSatFix(const Oem7MessageHeaderMem* header,
                                                          const BESTGNSSPOSMem* payload) {
    // Buffer to data struct
    NavSatFixData nav_sat_fix;
    NovToData(header, payload, nav_sat_fix);

    // Publish
    if (nav_sat_fix.frame_id == "GNSS1" || nav_sat_fix.frame_id == "GNSS") {
        if (navsatfix_gnss1_pub_.getNumSubscribers() > 0) {
            sensor_msgs::NavSatFix msg;
            NavSatFixDataToMsg(nav_sat_fix, msg);
            navsatfix_gnss1_pub_.publish(msg);
        }
    } else if (nav_sat_fix.frame_id == "GNSS2") {
        if (navsatfix_gnss2_pub_.getNumSubscribers() > 0) {
            sensor_msgs::NavSatFix msg;
            NavSatFixDataToMsg(nav_sat_fix, msg);
            navsatfix_gnss2_pub_.publish(msg);
        }
    }
}

}  // namespace fixposition

int main(int argc, char** argv) {
    ros::init(argc, argv, "fixposition_driver");
    ros::NodeHandle node_handle;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    fixposition::FixpositionDriverParams params;

    if (fixposition::LoadParamsFromRos1("~", params)) {
        ROS_INFO("Params Loaded!");
        fixposition::FixpositionDriverNode node(params);
        ROS_DEBUG("Starting node...");

        node.Run();
        ros::waitForShutdown();

        ROS_DEBUG("Exiting.");
        return 0;
    } else {
        ROS_ERROR("Params Loading Failed!");
        ros::shutdown();
        return 1;
    }
}
