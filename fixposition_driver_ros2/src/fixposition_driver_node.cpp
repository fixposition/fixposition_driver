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

/* SYSTEM / STL */
#include <memory>

/* ROS */
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

/* FIXPOSITION */
#include <fixposition_driver_lib/converter/imu.hpp>
#include <fixposition_driver_lib/converter/llh.hpp>
#include <fixposition_driver_lib/converter/odometry.hpp>
#include <fixposition_driver_lib/converter/tf.hpp>
#include <fixposition_driver_lib/converter/gpgga.hpp>
#include <fixposition_driver_lib/converter/gpzda.hpp>
#include <fixposition_driver_lib/converter/gprmc.hpp>
#include <fixposition_driver_lib/fixposition_driver.hpp>
#include <fixposition_driver_lib/helper.hpp>
#include <fixposition_gnss_tf/gnss_tf.hpp>

/* PACKAGE */
#include <fixposition_driver_ros2/data_to_ros2.hpp>
#include <fixposition_driver_ros2/fixposition_driver_node.hpp>
#include <fixposition_driver_ros2/params.hpp>

namespace fixposition {

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
            dynamic_cast<OdometryConverter*>(a_converters_["ODOMETRY"].get())
                ->AddObserver([this](const OdometryConverter::Msgs& data) {
                    // ODOMETRY Observer Lambda
                    // Msgs
                    if (odometry_pub_->get_subscription_count() > 0) {
                        nav_msgs::msg::Odometry odometry;
                        OdometryDataToMsg(data.odometry, odometry);
                        odometry_pub_->publish(odometry);
                    }

                    if (odometry_enu0_pub_->get_subscription_count() > 0) {
                        nav_msgs::msg::Odometry odometry_enu0;
                        OdometryDataToMsg(data.odometry_enu0, odometry_enu0);
                        odometry_enu0_pub_->publish(odometry_enu0);
                    }

                    if (vrtk_pub_->get_subscription_count() > 0) {
                        fixposition_driver_ros2::msg::VRTK vrtk;
                        VrtkDataToMsg(data.vrtk, vrtk);
                        vrtk_pub_->publish(vrtk);
                    }
                    if (eul_pub_->get_subscription_count() > 0) {
                        geometry_msgs::msg::Vector3Stamped ypr;
                        if (data.odometry.stamp.tow == 0.0 && data.odometry.stamp.wno == 0) {
                            ypr.header.stamp = rclcpp::Clock().now();
                        } else {
                            ypr.header.stamp = GpsTimeToMsgTime(data.odometry.stamp);
                        }
                        ypr.header.frame_id = "FP_POI";
                        ypr.vector.set__x(data.eul.x());
                        ypr.vector.set__y(data.eul.y());
                        ypr.vector.set__z(data.eul.z());
                        eul_pub_->publish(ypr);
                    }

                    if (poiimu_pub_->get_subscription_count() > 0) {
                        sensor_msgs::msg::Imu poiimu;
                        ImuDataToMsg(data.imu, poiimu);
                        poiimu_pub_->publish(poiimu);
                    }

                    // TFs
                    if (data.vrtk.fusion_status > 0) {
                        geometry_msgs::msg::TransformStamped tf_ecef_poi;
                        geometry_msgs::msg::TransformStamped tf_ecef_enu;
                        geometry_msgs::msg::TransformStamped tf_ecef_enu0;
                        TfDataToMsg(data.tf_ecef_poi, tf_ecef_poi);
                        TfDataToMsg(data.tf_ecef_enu, tf_ecef_enu);
                        TfDataToMsg(data.tf_ecef_enu0, tf_ecef_enu0);

                        br_->sendTransform(tf_ecef_enu);
                        br_->sendTransform(tf_ecef_poi);
                        static_br_->sendTransform(tf_ecef_enu0);
                    }
                });
        } else if (format == "ODOMSH") {
            dynamic_cast<OdometryConverter*>(a_converters_["ODOMSH"].get())
                ->AddObserver([this](const OdometryConverter::Msgs& data) {
                    if (odometry_smooth_pub_->get_subscription_count() > 0) {
                        nav_msgs::msg::Odometry odometry;
                        OdometryDataToMsg(data.odometry, odometry);
                        odometry_smooth_pub_->publish(odometry);
                    }
                });
        } else if (format == "LLH" && a_converters_["LLH"]) {
            dynamic_cast<LlhConverter*>(a_converters_["LLH"].get())->AddObserver([this](const NavSatFixData& data) {
                // LLH Observer Lambda
                sensor_msgs::msg::NavSatFix msg;
                NavSatFixDataToMsg(data, msg);
                navsatfix_pub_->publish(msg);
            });
        } else if (format == "RAWIMU") {
            dynamic_cast<ImuConverter*>(a_converters_["RAWIMU"].get())->AddObserver([this](const ImuData& data) {
                // RAWIMU Observer Lambda
                sensor_msgs::msg::Imu msg;
                ImuDataToMsg(data, msg);
                rawimu_pub_->publish(msg);
            });
        } else if (format == "CORRIMU") {
            dynamic_cast<ImuConverter*>(a_converters_["CORRIMU"].get())->AddObserver([this](const ImuData& data) {
                // CORRIMU Observer Lambda
                sensor_msgs::msg::Imu msg;
                ImuDataToMsg(data, msg);
                corrimu_pub_->publish(msg);
            });
        } else if (format == "TF") {
            dynamic_cast<TfConverter*>(a_converters_["TF"].get())->AddObserver([this](const TfData& data) {
                // TF Observer Lambda
                geometry_msgs::msg::TransformStamped tf;
                TfDataToMsg(data, tf);
                if (tf.child_frame_id == "FP_IMUH" && tf.header.frame_id == "FP_POI") {
                    br_->sendTransform(tf);

                    // Publish Pitch Roll based on IMU only
                    Eigen::Vector3d imu_ypr_eigen = gnss_tf::QuatToEul(data.rotation);
                    imu_ypr_eigen.x() = 0.0;  // the yaw value is not observable using IMU alone
                    geometry_msgs::msg::Vector3Stamped imu_ypr;
                    imu_ypr.header.stamp = tf.header.stamp;
                    imu_ypr.header.frame_id = "FP_POI";
                    imu_ypr.vector.set__x(imu_ypr_eigen.x());
                    imu_ypr.vector.set__y(imu_ypr_eigen.y());
                    imu_ypr.vector.set__z(imu_ypr_eigen.z());
                    eul_imu_pub_->publish(imu_ypr);

                } else {
                    static_br_->sendTransform(tf);
                }
            });
        } else if (format == "GPGGA") {
            dynamic_cast<GpggaConverter*>(a_converters_["GPGGA"].get())->AddObserver([this](const GpggaData& data) {
                // GPGGA Observer Lambda
                if (nmea_pub_->get_subscription_count() > 0) {
                    nmea_message_.gpgga = data;
                    PublishNmea(nmea_message_);
                }
            });
        } else if (format == "GPZDA") {
            dynamic_cast<GpzdaConverter*>(a_converters_["GPZDA"].get())->AddObserver([this](const GpzdaData& data) {
                // GPZDA Observer Lambda
                if (nmea_pub_->get_subscription_count() > 0) {
                    nmea_message_.gpzda = data;
                    PublishNmea(nmea_message_);
                }
            });
        } else if (format == "GPRMC") {
            dynamic_cast<GprmcConverter*>(a_converters_["GPRMC"].get())->AddObserver([this](const GprmcData& data) {
                // GPRMC Observer Lambda
                if (nmea_pub_->get_subscription_count() > 0) {
                    nmea_message_.gprmc = data;
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
        msg.header.frame_id = "LLH";

        // Latitude [degrees]. Positive is north of equator; negative is south
        msg.latitude = data.gpgga.latitude;

        // Longitude [degrees]. Positive is east of prime meridian; negative is west
        msg.longitude = data.gpgga.longitude;

        // Altitude [m]. Positive is above the WGS 84 ellipsoid
        msg.altitude = data.gpgga.altitude;

        // Speed over ground [m/s]
        msg.speed = data.gprmc.speed;

        // Course over ground [deg]
        msg.course = data.gprmc.course;

        // TODO: Get better position covariance from NMEA-GP-GST
        // Position covariance [m^2]
        Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map =
            Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.position_covariance.data());
        cov_map = data.gpgga.cov;
        msg.position_covariance_type = data.gpgga.position_covariance_type;

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
