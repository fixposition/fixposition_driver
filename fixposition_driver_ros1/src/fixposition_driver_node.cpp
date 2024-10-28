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

FixpositionDriverNode::FixpositionDriverNode(const FixpositionDriverParams& params)
    : FixpositionDriver(params),
      nh_("~"),
      // FP_A messages
      fpa_odometry_pub_(nh_.advertise<fixposition_driver_ros1::odometry>("/fixposition/fpa/odometry", 5)),
      fpa_imubias_pub_(nh_.advertise<fixposition_driver_ros1::imubias>("/fixposition/fpa/imubias", 5)),
      fpa_eoe_pub_(nh_.advertise<fixposition_driver_ros1::eoe>("/fixposition/fpa/eoe", 5)),
      fpa_llh_pub_(nh_.advertise<fixposition_driver_ros1::llh>("/fixposition/fpa/llh", 5)),
      fpa_odomenu_pub_(nh_.advertise<fixposition_driver_ros1::odomenu>("/fixposition/fpa/odomenu", 5)),
      fpa_odomsh_pub_(nh_.advertise<fixposition_driver_ros1::odomsh>("/fixposition/fpa/odomsh", 5)),
      fpa_odomstatus_pub_(nh_.advertise<fixposition_driver_ros1::odomstatus>("/fixposition/fpa/odomstatus", 5)),
      fpa_gnssant_pub_(nh_.advertise<fixposition_driver_ros1::gnssant>("/fixposition/fpa/gnssant", 5)),
      fpa_gnsscorr_pub_(nh_.advertise<fixposition_driver_ros1::gnsscorr>("/fixposition/fpa/gnsscorr", 5)),
      fpa_text_pub_(nh_.advertise<fixposition_driver_ros1::text>("/fixposition/fpa/text", 5)),
      fpa_tp_pub_(nh_.advertise<fixposition_driver_ros1::tp>("/fixposition/fpa/tp", 5)),

      // NMEA messages
      nmea_gpgga_pub_(nh_.advertise<fixposition_driver_ros1::gpgga>("/fixposition/nmea/gpgga", 5)),
      nmea_gpgll_pub_(nh_.advertise<fixposition_driver_ros1::gpgll>("/fixposition/nmea/gpgll", 5)),
      nmea_gngsa_pub_(nh_.advertise<fixposition_driver_ros1::gngsa>("/fixposition/nmea/gngsa", 5)),
      nmea_gpgst_pub_(nh_.advertise<fixposition_driver_ros1::gpgst>("/fixposition/nmea/gpgst", 5)),
      nmea_gxgsv_pub_(nh_.advertise<fixposition_driver_ros1::gxgsv>("/fixposition/nmea/gxgsv", 5)),
      nmea_gphdt_pub_(nh_.advertise<fixposition_driver_ros1::gphdt>("/fixposition/nmea/gphdt", 5)),
      nmea_gprmc_pub_(nh_.advertise<fixposition_driver_ros1::gprmc>("/fixposition/nmea/gprmc", 5)),
      nmea_gpvtg_pub_(nh_.advertise<fixposition_driver_ros1::gpvtg>("/fixposition/nmea/gpvtg", 5)),
      nmea_gpzda_pub_(nh_.advertise<fixposition_driver_ros1::gpzda>("/fixposition/nmea/gpzda", 5)),

      // ODOMETRY
      odometry_ecef_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_ecef", 5)),
      odometry_llh_pub_(nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/odometry_llh", 5)),
      odometry_enu_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_enu", 5)),
      odometry_smooth_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_smooth", 5)),

      // Orientation
      eul_pub_(nh_.advertise<geometry_msgs::Vector3Stamped>("/fixposition/ypr", 5)),
      eul_imu_pub_(nh_.advertise<geometry_msgs::Vector3Stamped>("/fixposition/imu_ypr", 5)),

      // IMU
      rawimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/rawimu", 5)),
      corrimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/corrimu", 5)),
      poiimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/poiimu", 5)),

      // GNSS
      nmea_pub_(nh_.advertise<fixposition_driver_ros1::NMEA>("/fixposition/nmea", 5)),
      navsatfix_gnss1_pub_(nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/gnss1", 5)),
      navsatfix_gnss2_pub_(nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/gnss2", 5))

{
    ws_sub_ = nh_.subscribe<fixposition_driver_ros1::Speed>(params_.customer_input.speed_topic, 10,
                                                            &FixpositionDriverNode::WsCallbackRos, this,
                                                            ros::TransportHints().tcpNoDelay());
    rtcm_sub_ = nh_.subscribe<rtcm_msgs::Message>(params_.customer_input.rtcm_topic, 10,
                                                  &FixpositionDriverNode::RtcmCallbackRos, this,
                                                  ros::TransportHints().tcpNoDelay());

    // Configure jump warning message
    if (params_.fp_output.cov_warning) {
        extras_jump_pub_ = nh_.advertise<fixposition_driver_ros1::CovWarn>("/fixposition/extras/jump", 5);
        prev_pos.setZero();
        prev_cov.setZero();
    }
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

                    // Output jump warning
                    if (params_.fp_output.cov_warning) {
                        if (!prev_pos.isZero() && !prev_cov.isZero()) {
                            Eigen::Vector3d pos_diff = (prev_pos - data.odom.pose.position).cwiseAbs();

                            if ((pos_diff[0] > prev_cov(0,0)) || (pos_diff[1] > prev_cov(1,1)) || (pos_diff[2] > prev_cov(2,2))) {
                                JumpWarningMsg(data.odom.stamp, pos_diff, prev_cov, extras_jump_pub_);
                            }
                        }
                        prev_pos = data.odom.pose.position;
                        prev_cov = data.odom.pose.cov;
                    }
                });
        } else if (format == "ODOMENU") {
            dynamic_cast<NmeaConverter<FP_ODOMENU>*>(a_converters_["ODOMENU"].get())
                ->AddObserver([this](const FP_ODOMENU& data) {
                    FpToRosMsg(data, fpa_odomenu_pub_);
                    FpToRosMsg(data.odom, odometry_enu_pub_);
                    OdomToYprMsg(data.odom, eul_pub_);

                    // Append TF if Nav2 mode is selected
                    if (params_.fp_output.nav2_mode) {
                        // Get FP_ENU0 -> FP_POI
                        geometry_msgs::TransformStamped tf;
                        OdomToTf(data.odom, tf);
                        tf_map["ENU0POI"] = std::make_shared<geometry_msgs::TransformStamped>(tf);
                    }
                });
        } else if (format == "ODOMSH") {
            dynamic_cast<NmeaConverter<FP_ODOMSH>*>(a_converters_["ODOMSH"].get())
                ->AddObserver([this](const FP_ODOMSH& data) {
                    FpToRosMsg(data, fpa_odomsh_pub_);
                    FpToRosMsg(data.odom, odometry_smooth_pub_);

                    // Append TF if Nav2 mode is selected
                    if (params_.fp_output.nav2_mode) {
                        // Get FP_ECEF -> FP_POISH
                        geometry_msgs::TransformStamped tf;
                        OdomToTf(data.odom, tf);
                        tf_map["ECEFPOISH"] = std::make_shared<geometry_msgs::TransformStamped>(tf);
                    }
                });
        } else if (format == "ODOMSTATUS") {
            dynamic_cast<NmeaConverter<FP_ODOMSTATUS>*>(a_converters_["ODOMSTATUS"].get())
                ->AddObserver([this](const FP_ODOMSTATUS& data) { FpToRosMsg(data, fpa_odomstatus_pub_); });
        } else if (format == "IMUBIAS") {
            dynamic_cast<NmeaConverter<FP_IMUBIAS>*>(a_converters_["IMUBIAS"].get())
                ->AddObserver([this](const FP_IMUBIAS& data) { FpToRosMsg(data, fpa_imubias_pub_); });
        } else if (format == "EOE") {
            dynamic_cast<NmeaConverter<FP_EOE>*>(a_converters_["EOE"].get())
                ->AddObserver([this](const FP_EOE& data) {
                    FpToRosMsg(data, fpa_eoe_pub_);

                    // Generate Nav2 TF tree
                    if (data.epoch == "FUSION" && params_.fp_output.nav2_mode) {
                        PublishNav2Tf(tf_map, static_br_, br_);
                    }
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
                if (data.valid_tf) {
                    // TF Observer Lambda
                    geometry_msgs::TransformStamped tf;
                    TfDataToMsg(data.tf, tf);
                    if (tf.child_frame_id == "FP_IMUH" && tf.header.frame_id == "FP_POI") {
                        br_.sendTransform(tf);

                        // Publish Pitch Roll based on IMU only
                        Eigen::Vector3d imu_ypr_eigen = QuatToEul(data.tf.rotation);
                        imu_ypr_eigen.x() = 0.0;  // the yaw value is not observable using IMU alone
                        geometry_msgs::Vector3Stamped imu_ypr;
                        imu_ypr.header.stamp = tf.header.stamp;
                        imu_ypr.header.frame_id = "FP_POI";
                        tf::vectorEigenToMsg(imu_ypr_eigen, imu_ypr.vector);
                        eul_imu_pub_.publish(imu_ypr);

                    } else if (tf.child_frame_id == "FP_POISH" && tf.header.frame_id == "FP_POI") {
                        br_.sendTransform(tf);

                        // Append TF if Nav2 mode is selected
                        if (params_.fp_output.nav2_mode) {
                            // Get FP_POI -> FP_POISH
                            tf_map["POIPOISH"] = std::make_shared<geometry_msgs::TransformStamped>(tf);
                        }
                    } else if (tf.child_frame_id == "FP_ENU0" && tf.header.frame_id == "FP_ECEF") {
                        static_br_.sendTransform(tf);

                        // Append TF if Nav2 mode is selected
                        if (params_.fp_output.nav2_mode) {
                            // Get FP_ECEF -> FP_ENU0
                            tf_map["ECEFENU0"] = std::make_shared<geometry_msgs::TransformStamped>(tf);
                        }
                    } else {
                        static_br_.sendTransform(tf);
                    }
                }
            });
        } else if (format == "TP") {
            dynamic_cast<NmeaConverter<FP_TP>*>(a_converters_["TP"].get())
                ->AddObserver([this](const FP_TP& data) { FpToRosMsg(data, fpa_tp_pub_); });
        } else if (format == "GPGGA") {
            dynamic_cast<NmeaConverter<GP_GGA>*>(a_converters_["GPGGA"].get())->AddObserver([this](const GP_GGA& data) {
                FpToRosMsg(data, nmea_gpgga_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                    PublishNmea();  // GPGGA controls the NMEA output
                }
            });
        } else if (format == "GPGLL") {
            dynamic_cast<NmeaConverter<GP_GLL>*>(a_converters_["GPGLL"].get())->AddObserver([this](const GP_GLL& data) {
                FpToRosMsg(data, nmea_gpgll_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GNGSA") {
            dynamic_cast<NmeaConverter<GN_GSA>*>(a_converters_["GNGSA"].get())->AddObserver([this](const GN_GSA& data) {
                FpToRosMsg(data, nmea_gngsa_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPGST") {
            dynamic_cast<NmeaConverter<GP_GST>*>(a_converters_["GPGST"].get())->AddObserver([this](const GP_GST& data) {
                FpToRosMsg(data, nmea_gpgst_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GXGSV") {
            dynamic_cast<NmeaConverter<GX_GSV>*>(a_converters_["GXGSV"].get())->AddObserver([this](const GX_GSV& data) {
                FpToRosMsg(data, nmea_gxgsv_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPHDT") {
            dynamic_cast<NmeaConverter<GP_HDT>*>(a_converters_["GPHDT"].get())->AddObserver([this](const GP_HDT& data) {
                FpToRosMsg(data, nmea_gphdt_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPRMC") {
            dynamic_cast<NmeaConverter<GP_RMC>*>(a_converters_["GPRMC"].get())->AddObserver([this](const GP_RMC& data) {
                FpToRosMsg(data, nmea_gprmc_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPVTG") {
            dynamic_cast<NmeaConverter<GP_VTG>*>(a_converters_["GPVTG"].get())->AddObserver([this](const GP_VTG& data) {
                FpToRosMsg(data, nmea_gpvtg_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPZDA") {
            dynamic_cast<NmeaConverter<GP_ZDA>*>(a_converters_["GPZDA"].get())->AddObserver([this](const GP_ZDA& data) {
                FpToRosMsg(data, nmea_gpzda_pub_);
                if (nmea_pub_.getNumSubscribers() > 0) nmea_message_.AddNmeaEpoch(data);
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
        for (auto gsv_it = nmea_message_.gnss_signals.begin(); gsv_it != nmea_message_.gnss_signals.end(); ++gsv_it) {
            SignalType msg_type = gsv_it->first;
            std::map<unsigned int, GnssSignalStats>* gnss_data = &gsv_it->second;

            // Populate GnssSats message
            fixposition_driver_ros1::GnssSats sats_msg;

            // Get constellation name
            if (msg_type == SignalType::GPS) {
                sats_msg.constellation = "GPS";
            } else if (msg_type == SignalType::Galileo) {
                sats_msg.constellation = "Galileo";
            } else if (msg_type == SignalType::BeiDou) {
                sats_msg.constellation = "BeiDou";
            } else if (msg_type == SignalType::GLONASS) {
                sats_msg.constellation = "GLONASS";
            } else {
                sats_msg.constellation = "Unknown";
            }

            // Get signal statistics
            for (auto it = gnss_data->begin(); it != gnss_data->end(); ++it) {
                unsigned int sat_id = it->first;
                GnssSignalStats signals = it->second;

                sats_msg.sat_id.push_back(sat_id);
                sats_msg.azim.push_back(signals.azim);
                sats_msg.elev.push_back(signals.elev);
                sats_msg.cno_l1.push_back(signals.cno_l1);
                sats_msg.cno_l2.push_back(signals.cno_l2);
            }

            // Add GnssSats to NMEA message
            msg.gnss_sats.push_back(sats_msg);
        }

        // Clear map
        nmea_message_.gnss_signals.clear();

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

void FixpositionDriverNode::WsCallbackRos(const fixposition_driver_ros1::SpeedConstPtr& msg) {
    std::unordered_map<std::string, std::vector<std::pair<bool, int>>> measurements;
    for (const auto& sensor : msg->sensors) {
        measurements[sensor.location].push_back({sensor.vx_valid, sensor.vx});
        measurements[sensor.location].push_back({sensor.vy_valid, sensor.vy});
        measurements[sensor.location].push_back({sensor.vz_valid, sensor.vz});
    }
    WsCallback(measurements);
}

void FixpositionDriverNode::RtcmCallbackRos(const rtcm_msgs::MessageConstPtr& msg) {
    const void* rtcm_msg = &(msg->message[0]);
    size_t msg_size = msg->message.size();
    RtcmCallback(rtcm_msg, msg_size);
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
