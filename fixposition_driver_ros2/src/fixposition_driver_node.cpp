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

// auto qos_sensor = rclcpp::QoS(
//     rclcpp::QoSInitialization(
//         rmw_qos_profile_sensor_data.history,
//         rmw_qos_profile_sensor_data.depth,
//     ),
//     rmw_qos_profile_sensor_data
// );

FixpositionDriverNode::FixpositionDriverNode(std::shared_ptr<rclcpp::Node> node, const FixpositionDriverParams& params)
    : FixpositionDriver(params),
      node_(node),
      rawimu_pub_(node_->create_publisher<sensor_msgs::msg::Imu>("/fixposition/rawimu", 100)),
      corrimu_pub_(node_->create_publisher<sensor_msgs::msg::Imu>("/fixposition/corrimu", 100)),
      navsatfix_pub_(node_->create_publisher<sensor_msgs::msg::NavSatFix>("/fixposition/navsatfix", 100)),
      navsatfix_gnss1_pub_(node_->create_publisher<sensor_msgs::msg::NavSatFix>("/fixposition/gnss1", 100)),
      navsatfix_gnss2_pub_(node_->create_publisher<sensor_msgs::msg::NavSatFix>("/fixposition/gnss2", 100)),
      nmea_pub_(node_->create_publisher<fixposition_driver_ros2::msg::NMEA>("/fixposition/nmea", 100)),
      odometry_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry", 100)),
      odometry_smooth_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odomsh", 100)),
      poiimu_pub_(node_->create_publisher<sensor_msgs::msg::Imu>("/fixposition/poiimu", 100)),
      vrtk_pub_(node_->create_publisher<fixposition_driver_ros2::msg::VRTK>("/fixposition/vrtk", 100)),
      odometry_enu0_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry_enu", 100)),
      eul_pub_(node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/fixposition/ypr", 100)),
      eul_imu_pub_(node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/fixposition/imu_ypr", 100)),
      br_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
      static_br_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_)) {
    ws_sub_ = node_->create_subscription<fixposition_driver_ros2::msg::Speed>(
        params_.customer_input.speed_topic, 100,
        std::bind(&FixpositionDriverNode::WsCallback, this, std::placeholders::_1));

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
            rate.sleep();
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
                    // ODOMETRY Observer Lambda
                    if (odometry_pub_->get_subscription_count() > 0) {
                        nav_msgs::msg::Odometry odometry;
                        OdometryDataToMsg(data.odom, odometry);
                        odometry_pub_->publish(odometry);
                    }

                    if (vrtk_pub_->get_subscription_count() > 0) {
                        fixposition_driver_ros2::msg::VRTK vrtk;
                        OdomToVrtkMsg(data, vrtk);
                        vrtk_pub_->publish(vrtk);
                    }

                    if (poiimu_pub_->get_subscription_count() > 0) {
                        sensor_msgs::msg::Imu poiimu;
                        OdomToImuMsg(data, poiimu);
                        poiimu_pub_->publish(poiimu);
                    }

                    if (navsatfix_pub_->get_subscription_count() > 0) {
                        sensor_msgs::msg::NavSatFix msg;
                        OdomToNavSatFix(data, msg);
                        navsatfix_pub_->publish(msg);
                    }

                    // TFs
                    if (data.fusion_status > 0) {
                        geometry_msgs::msg::TransformStamped tf_ecef_poi;
                        OdometryDataToTf(data.odom, tf_ecef_poi);
                        br_->sendTransform(tf_ecef_poi);
                    }
                });
        } else if (format == "ODOMENU") {
            dynamic_cast<NmeaConverter<FP_ODOMENU>*>(a_converters_["ODOMENU"].get())
                ->AddObserver([this](const FP_ODOMENU& data) {
                    // ODOMENU Observer Lambda
                    if (odometry_enu0_pub_->get_subscription_count() > 0) {
                        nav_msgs::msg::Odometry odometry_enu0;
                        OdometryDataToMsg(data.odom, odometry_enu0);
                        odometry_enu0_pub_->publish(odometry_enu0);
                    }

                    if (eul_pub_->get_subscription_count() > 0) {
                        geometry_msgs::msg::Vector3Stamped ypr;
                        if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
                            ypr.header.stamp = rclcpp::Clock().now();
                        } else {
                            ypr.header.stamp = GpsTimeToMsgTime(data.odom.stamp);
                        }
                        ypr.header.frame_id = "FP_ENU";

                        // Euler angle wrt. ENU frame in the order of Yaw Pitch Roll
                        Eigen::Vector3d enu_euler = gnss_tf::RotToEul(data.odom.pose.orientation.toRotationMatrix());
                        ypr.vector.set__x(enu_euler.x());
                        ypr.vector.set__y(enu_euler.y());
                        ypr.vector.set__z(enu_euler.z());
                        eul_pub_->publish(ypr);
                    }
                });
        } else if (format == "ODOMSH") {
            dynamic_cast<NmeaConverter<FP_ODOMSH>*>(a_converters_["ODOMSH"].get())
                ->AddObserver([this](const FP_ODOMSH& data) {
                    // ODOMSH Observer Lambda
                    if (odometry_smooth_pub_->get_subscription_count() > 0) {
                        nav_msgs::msg::Odometry odometry;
                        OdometryDataToMsg(data.odom, odometry);
                        odometry_smooth_pub_->publish(odometry);
                    }
                });
        } else if (format == "RAWIMU") {
            dynamic_cast<NmeaConverter<FP_RAWIMU>*>(a_converters_["RAWIMU"].get())->AddObserver([this](const FP_RAWIMU& data) {
                // RAWIMU Observer Lambda
                sensor_msgs::msg::Imu msg;
                ImuDataToMsg(data.imu, msg);
                rawimu_pub_->publish(msg);
            });
        } else if (format == "CORRIMU") {
            dynamic_cast<NmeaConverter<FP_CORRIMU>*>(a_converters_["CORRIMU"].get())->AddObserver([this](const FP_CORRIMU& data) {
                // CORRIMU Observer Lambda
                sensor_msgs::msg::Imu msg;
                ImuDataToMsg(data.imu, msg);
                corrimu_pub_->publish(msg);
            });
        } else if (format == "TF") {
            dynamic_cast<NmeaConverter<FP_TF>*>(a_converters_["TF"].get())->AddObserver([this](const FP_TF& data) {
                // TF Observer Lambda
                geometry_msgs::msg::TransformStamped tf;
                TfDataToMsg(data.tf, tf);
                if (tf.child_frame_id == "FP_IMUH" && tf.header.frame_id == "FP_POI") {
                    br_->sendTransform(tf);

                    // Publish Pitch Roll based on IMU only
                    Eigen::Vector3d imu_ypr_eigen = gnss_tf::QuatToEul(data.tf.rotation);
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
                } else {
                    static_br_->sendTransform(tf);
                }
            });
        } else if (format == "GPGGA") {
            dynamic_cast<NmeaConverter<GP_GGA>*>(a_converters_["GPGGA"].get())->AddObserver([this](const GP_GGA& data) {
                // GPGGA Observer Lambda
                if (nmea_pub_->get_subscription_count() > 0) {
                    nmea_message_.gpgga = data;
                    PublishNmea(nmea_message_);
                }
            });
        } else if (format == "GPRMC") {
            dynamic_cast<NmeaConverter<GP_RMC>*>(a_converters_["GPRMC"].get())->AddObserver([this](const GP_RMC& data) {
                // GPRMC Observer Lambda
                if (nmea_pub_->get_subscription_count() > 0) {
                    nmea_message_.gprmc = data;
                    PublishNmea(nmea_message_);
                }
            });
        } else if (format == "GPZDA") {
            dynamic_cast<NmeaConverter<GP_ZDA>*>(a_converters_["GPZDA"].get())->AddObserver([this](const GP_ZDA& data) {
                // GPZDA Observer Lambda
                if (nmea_pub_->get_subscription_count() > 0) {
                    nmea_message_.gpzda = data;
                    PublishNmea(nmea_message_);
                }
            });
        }
    }
}

void FixpositionDriverNode::PublishNmea(NmeaMessage data) {
    // If epoch message is complete, generate NMEA output
    if (data.checkEpoch()) {
        // Generate new message
        fixposition_driver_ros2::msg::NMEA msg;

        // ROS Header
        if (data.gpzda.stamp.tow == 0.0 && data.gpzda.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.gpzda.stamp);
        }
        msg.header.frame_id = "FP_POI";

        // Latitude [degrees]. Positive is north of equator; negative is south
        msg.latitude = data.gpgga.llh(0);

        // Longitude [degrees]. Positive is east of prime meridian; negative is west
        msg.longitude = data.gpgga.llh(1);

        // Altitude [m]. Positive is above the WGS 84 ellipsoid
        msg.altitude = data.gpgga.llh(2);

        // Speed over ground [m/s]
        msg.speed = data.gprmc.speed;

        // Course over ground [deg]
        msg.course = data.gprmc.course;

        // TODO: Get better position covariance from NMEA-GP-GST
        // Position covariance [m^2]
        Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map =
            Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.position_covariance.data());
        
        // Covariance diagonals
        Eigen::Matrix<double, 3, 3> gpgga_cov;
        const double hdop = data.gpgga.hdop;
        gpgga_cov(0, 0) = hdop * hdop;
        gpgga_cov(1, 1) = hdop * hdop;
        gpgga_cov(2, 2) = 4 * hdop * hdop;

        // Rest of covariance fields
        gpgga_cov(0, 1) = gpgga_cov(1, 0) = 0.0;
        gpgga_cov(0, 2) = gpgga_cov(2, 0) = 0.0;
        gpgga_cov(1, 2) = gpgga_cov(2, 1) = 0.0;
        
        cov_map = gpgga_cov;
        msg.position_covariance_type = 1; // COVARIANCE_TYPE_APPROXIMATED

        // Positioning system mode indicator, R (RTK fixed), F (RTK float), A (no RTK), E, N
        msg.mode = data.gprmc.mode;

        // Publish message
        nmea_pub_->publish(msg);
    }
}

void FixpositionDriverNode::WsCallback(const fixposition_driver_ros2::msg::Speed::ConstSharedPtr msg) {
    std::unordered_map<std::string, std::vector<std::pair<bool, int>>> measurements;
    for (const auto &sensor : msg->sensors) {
        measurements[sensor.location].push_back({sensor.vx_valid, sensor.vx});
        measurements[sensor.location].push_back({sensor.vy_valid, sensor.vy});
        measurements[sensor.location].push_back({sensor.vz_valid, sensor.vz});
    }
    FixpositionDriver::WsCallback(measurements);
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
        fixposition::FixpositionDriverNode driver_node(node, params);
        driver_node.Run();
        RCLCPP_INFO(node->get_logger(), "Exiting.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Params Loading Failed!");
        rclcpp::shutdown();
        return 1;
    }
}
