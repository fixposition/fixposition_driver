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
#include <fixposition_driver_ros2/fixposition_driver_node.hpp>

namespace fixposition {

FixpositionDriverNode::FixpositionDriverNode(std::shared_ptr<rclcpp::Node> node, const FixpositionDriverParams& params, rclcpp::QoS qos_settings)
    : FixpositionDriver(params),
      node_(node),

      // FP_A messages
      fpa_odometry_pub_(node_->create_publisher<fixposition_driver_ros2::msg::ODOMETRY>("/fixposition/fpa/odometry", qos_settings)),
      fpa_imubias_pub_(node_->create_publisher<fixposition_driver_ros2::msg::IMUBIAS>("/fixposition/fpa/imubias", qos_settings)),
      fpa_eoe_pub_(node_->create_publisher<fixposition_driver_ros2::msg::EOE>("/fixposition/fpa/eoe", qos_settings)),
      fpa_llh_pub_(node_->create_publisher<fixposition_driver_ros2::msg::LLH>("/fixposition/fpa/llh", qos_settings)),
      fpa_odomenu_pub_(node_->create_publisher<fixposition_driver_ros2::msg::ODOMENU>("/fixposition/fpa/odomenu", qos_settings)),
      fpa_odomsh_pub_(node_->create_publisher<fixposition_driver_ros2::msg::ODOMSH>("/fixposition/fpa/odomsh", qos_settings)),
      fpa_odomstatus_pub_(node_->create_publisher<fixposition_driver_ros2::msg::ODOMSTATUS>("/fixposition/fpa/odomstatus", qos_settings)),
      fpa_gnssant_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GNSSANT>("/fixposition/fpa/gnssant", qos_settings)),
      fpa_gnsscorr_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GNSSCORR>("/fixposition/fpa/gnsscorr", qos_settings)),
      fpa_text_pub_(node_->create_publisher<fixposition_driver_ros2::msg::TEXT>("/fixposition/fpa/text", qos_settings)),
      fpa_tp_pub_(node_->create_publisher<fixposition_driver_ros2::msg::TP>("/fixposition/fpa/tp", qos_settings)),

      // NMEA messages
      nmea_gpgga_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GPGGA>("/fixposition/nmea/gpgga", qos_settings)),
      nmea_gpgll_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GPGLL>("/fixposition/nmea/gpgll", qos_settings)),
      nmea_gngsa_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GNGSA>("/fixposition/nmea/gngsa", qos_settings)),
      nmea_gpgst_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GPGST>("/fixposition/nmea/gpgst", qos_settings)),
      nmea_gxgsv_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GXGSV>("/fixposition/nmea/gxgsv", qos_settings)),
      nmea_gphdt_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GPHDT>("/fixposition/nmea/gphdt", qos_settings)),
      nmea_gprmc_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GPRMC>("/fixposition/nmea/gprmc", qos_settings)),
      nmea_gpvtg_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GPVTG>("/fixposition/nmea/gpvtg", qos_settings)),
      nmea_gpzda_pub_(node_->create_publisher<fixposition_driver_ros2::msg::GPZDA>("/fixposition/nmea/gpzda", qos_settings)),

      // ODOMETRY
      odometry_ecef_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry_ecef", qos_settings)),
      odometry_llh_pub_(node_->create_publisher<sensor_msgs::msg::NavSatFix>("/fixposition/odometry_llh", qos_settings)),
      odometry_enu_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry_enu", qos_settings)),
      odometry_smooth_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry_smooth", qos_settings)),

      // Orientation
      eul_pub_(node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/fixposition/ypr", qos_settings)),
      eul_imu_pub_(node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/fixposition/imu_ypr", qos_settings)),

      // IMU
      rawimu_pub_(node_->create_publisher<sensor_msgs::msg::Imu>("/fixposition/rawimu", qos_settings)),
      corrimu_pub_(node_->create_publisher<sensor_msgs::msg::Imu>("/fixposition/corrimu", qos_settings)),
      poiimu_pub_(node_->create_publisher<sensor_msgs::msg::Imu>("/fixposition/poiimu", qos_settings)),

      // GNSS
      nmea_pub_(node_->create_publisher<fixposition_driver_ros2::msg::NMEA>("/fixposition/nmea", qos_settings)),
      navsatfix_gnss1_pub_(node_->create_publisher<sensor_msgs::msg::NavSatFix>("/fixposition/gnss1", qos_settings)),
      navsatfix_gnss2_pub_(node_->create_publisher<sensor_msgs::msg::NavSatFix>("/fixposition/gnss2", qos_settings)),

      // TF
      br_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
      static_br_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_))

{
    ws_sub_ = node_->create_subscription<fixposition_driver_ros2::msg::Speed>(
        params_.customer_input.speed_topic, 100,
        std::bind(&FixpositionDriverNode::WsCallback, this, std::placeholders::_1));
    rtcm_sub_ = node_->create_subscription<fixposition_driver_ros2::msg::RTCM>(
        params_.customer_input.rtcm_topic, 10,
        std::bind(&FixpositionDriverNode::RtcmCallback, this, std::placeholders::_1));

    // Configure jump warning message
    if (params_.fp_output.cov_warning) {
        extras_jump_pub_ = node_->create_publisher<fixposition_driver_ros2::msg::COVWARN>("/fixposition/extras/jump", qos_settings);
        prev_pos.setZero();
        prev_cov.setZero();
    }
    RegisterObservers();
}

void FixpositionDriverNode::Run() {
    rclcpp::Rate rate(params_.fp_output.rate);
    const auto reconnect_delay =
        std::chrono::nanoseconds((uint64_t)params_.fp_output.reconnect_delay * 1000 * 1000 * 1000);

    while (rclcpp::ok()) {
        // Read data and publish to ros
        const bool connection_ok = RunOnce();
        // process Incoming ROS msgs
        rclcpp::spin_some(node_);
        // Handle connection loss
        if (!connection_ok) {
            printf("Reconnecting in %.1f seconds ...\n", params_.fp_output.reconnect_delay);
            rclcpp::sleep_for(reconnect_delay);
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
                                JumpWarningMsg(node_, data.odom.stamp, pos_diff, prev_cov, extras_jump_pub_);
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
                        geometry_msgs::msg::TransformStamped tf;
                        OdomToTf(data.odom, tf);
                        tf_map["ENU0POI"] = std::make_shared<geometry_msgs::msg::TransformStamped>(tf);
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
                        geometry_msgs::msg::TransformStamped tf;
                        OdomToTf(data.odom, tf);
                        tf_map["ECEFPOISH"] = std::make_shared<geometry_msgs::msg::TransformStamped>(tf);
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
                    geometry_msgs::msg::TransformStamped tf;
                    TfDataToMsg(data.tf, tf);
                    if (tf.child_frame_id == "FP_IMUH" && tf.header.frame_id == "FP_POI") {
                        br_->sendTransform(tf);

                        // Publish Pitch Roll based on IMU only
                        Eigen::Vector3d imu_ypr_eigen = QuatToEul(data.tf.rotation);
                        imu_ypr_eigen.x() = 0.0;  // the yaw value is not observable using IMU alone
                        geometry_msgs::msg::Vector3Stamped imu_ypr;
                        imu_ypr.header.stamp = tf.header.stamp;
                        imu_ypr.header.frame_id = "FP_POI";
                        imu_ypr.vector.set__x(imu_ypr_eigen.x());
                        imu_ypr.vector.set__y(imu_ypr_eigen.y());
                        imu_ypr.vector.set__z(imu_ypr_eigen.z());
                        eul_imu_pub_->publish(imu_ypr);

                    } else if (tf.child_frame_id == "FP_POISH" && tf.header.frame_id == "FP_POI") {
                        br_->sendTransform(tf);

                        // Append TF if Nav2 mode is selected
                        if (params_.fp_output.nav2_mode) {
                            // Get FP_POI -> FP_POISH
                            tf_map["POIPOISH"] = std::make_shared<geometry_msgs::msg::TransformStamped>(tf);
                        }
                    } else if (tf.child_frame_id == "FP_ENU0" && tf.header.frame_id == "FP_ECEF") {
                        static_br_->sendTransform(tf);

                        // Append TF if Nav2 mode is selected
                        if (params_.fp_output.nav2_mode) {
                            // Get FP_ECEF -> FP_ENU0
                            tf_map["ECEFENU0"] = std::make_shared<geometry_msgs::msg::TransformStamped>(tf);
                        }
                    } else {
                        static_br_->sendTransform(tf);
                    }
                }
            });
        } else if (format == "TP") {
            dynamic_cast<NmeaConverter<FP_TP>*>(a_converters_["TP"].get())
                ->AddObserver([this](const FP_TP& data) { FpToRosMsg(data, fpa_tp_pub_); });
        } else if (format == "GPGGA") {
            dynamic_cast<NmeaConverter<GP_GGA>*>(a_converters_["GPGGA"].get())->AddObserver([this](const GP_GGA& data) {
                FpToRosMsg(data, nmea_gpgga_pub_);
                if (nmea_pub_->get_subscription_count() > 0) {
                    nmea_message_.AddNmeaEpoch(data);
                    PublishNmea();  // GPGGA controls the NMEA output
                }
            });
        } else if (format == "GPGLL") {
            dynamic_cast<NmeaConverter<GP_GLL>*>(a_converters_["GPGLL"].get())->AddObserver([this](const GP_GLL& data) {
                FpToRosMsg(data, nmea_gpgll_pub_);
                if (nmea_pub_->get_subscription_count() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GNGSA") {
            dynamic_cast<NmeaConverter<GN_GSA>*>(a_converters_["GNGSA"].get())->AddObserver([this](const GN_GSA& data) {
                FpToRosMsg(data, nmea_gngsa_pub_);
                if (nmea_pub_->get_subscription_count() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPGST") {
            dynamic_cast<NmeaConverter<GP_GST>*>(a_converters_["GPGST"].get())->AddObserver([this](const GP_GST& data) {
                FpToRosMsg(data, nmea_gpgst_pub_);
                if (nmea_pub_->get_subscription_count() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GXGSV") {
            dynamic_cast<NmeaConverter<GX_GSV>*>(a_converters_["GXGSV"].get())->AddObserver([this](const GX_GSV& data) {
                FpToRosMsg(data, nmea_gxgsv_pub_);
                if (nmea_pub_->get_subscription_count() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPHDT") {
            dynamic_cast<NmeaConverter<GP_HDT>*>(a_converters_["GPHDT"].get())->AddObserver([this](const GP_HDT& data) {
                FpToRosMsg(data, nmea_gphdt_pub_);
                if (nmea_pub_->get_subscription_count() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPRMC") {
            dynamic_cast<NmeaConverter<GP_RMC>*>(a_converters_["GPRMC"].get())->AddObserver([this](const GP_RMC& data) {
                FpToRosMsg(data, nmea_gprmc_pub_);
                if (nmea_pub_->get_subscription_count() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPVTG") {
            dynamic_cast<NmeaConverter<GP_VTG>*>(a_converters_["GPVTG"].get())->AddObserver([this](const GP_VTG& data) {
                FpToRosMsg(data, nmea_gpvtg_pub_);
                if (nmea_pub_->get_subscription_count() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        } else if (format == "GPZDA") {
            dynamic_cast<NmeaConverter<GP_ZDA>*>(a_converters_["GPZDA"].get())->AddObserver([this](const GP_ZDA& data) {
                FpToRosMsg(data, nmea_gpzda_pub_);
                if (nmea_pub_->get_subscription_count() > 0) nmea_message_.AddNmeaEpoch(data);
            });
        }
    }
}

void FixpositionDriverNode::PublishNmea() {
    // If epoch message is complete, generate NMEA output
    if (nmea_message_.checkEpoch()) {
        // Generate new message
        fixposition_driver_ros2::msg::NMEA msg;

        // ROS Header
        if (nmea_message_.stamp.tow == 0.0 && nmea_message_.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(nmea_message_.stamp);
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
            fixposition_driver_ros2::msg::Gnsssats sats_msg;

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
        nmea_pub_->publish(msg);
    }
}

void FixpositionDriverNode::WsCallback(const fixposition_driver_ros2::msg::Speed::ConstSharedPtr msg) {
    std::unordered_map<std::string, std::vector<std::pair<bool, int>>> measurements;
    for (const auto& sensor : msg->sensors) {
        measurements[sensor.location].push_back({sensor.vx_valid, sensor.vx});
        measurements[sensor.location].push_back({sensor.vy_valid, sensor.vy});
        measurements[sensor.location].push_back({sensor.vz_valid, sensor.vz});
    }
    FixpositionDriver::WsCallback(measurements);
}

void FixpositionDriverNode::RtcmCallback(const fixposition_driver_ros2::msg::RTCM::ConstSharedPtr msg) {
    const void* rtcm_msg = &(msg->message[0]);
    size_t msg_size = msg->message.size();
    FixpositionDriver::RtcmCallback(rtcm_msg, msg_size);
}

void FixpositionDriverNode::BestGnssPosToPublishNavSatFix(const Oem7MessageHeaderMem* header,
                                                          const BESTGNSSPOSMem* payload) {
    // Buffer to data struct
    NavSatFixData nav_sat_fix;
    NovToData(header, payload, nav_sat_fix);

    // Publish
    if (nav_sat_fix.frame_id == "GNSS1" || nav_sat_fix.frame_id == "GNSS") {
        if (navsatfix_gnss1_pub_->get_subscription_count() > 0) {
            sensor_msgs::msg::NavSatFix msg;
            NavSatFixDataToMsg(nav_sat_fix, msg);
            navsatfix_gnss1_pub_->publish(msg);
        }
    } else if (nav_sat_fix.frame_id == "GNSS2") {
        if (navsatfix_gnss2_pub_->get_subscription_count() > 0) {
            sensor_msgs::msg::NavSatFix msg;
            NavSatFixDataToMsg(nav_sat_fix, msg);
            navsatfix_gnss2_pub_->publish(msg);
        }
    }
}

}  // namespace fixposition

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fixposition_driver");
    fixposition::FixpositionDriverParams params;

    RCLCPP_INFO(node->get_logger(), "Starting node...");

    if (fixposition::LoadParamsFromRos2(node, params)) {
        RCLCPP_INFO(node->get_logger(), "Params Loaded!");
        
        // Define ROS QoS
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default);

        if (params.fp_output.qos_type == "sensor_short") { // Short-queue sensor-type QoS
            qos_settings = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data);
        } else if (params.fp_output.qos_type == "sensor_long") { // Long-queue sensor-type QoS
            qos_settings = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
        } else if (params.fp_output.qos_type == "default_short") { // Short-queue default-type QoS
            qos_settings = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_default);
        } else if (params.fp_output.qos_type == "default_long") { // Long-queue default-type QoS
            qos_settings = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default);
        } else { // Default QoS profile
            qos_settings = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default);
        }

        fixposition::FixpositionDriverNode driver_node(node, params, qos_settings);
        driver_node.Run();
        RCLCPP_INFO(node->get_logger(), "Exiting.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Params Loading Failed!");
        rclcpp::shutdown();
        return 1;
    }
}
