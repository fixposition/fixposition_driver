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

/* ROS */
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <ros/ros.h>

/* FIXPOSITION */
#include <fixposition_driver_lib/converter/imu.hpp>
#include <fixposition_driver_lib/converter/odometry.hpp>
#include <fixposition_driver_lib/converter/odomenu.hpp>
#include <fixposition_driver_lib/converter/odomsh.hpp>
#include <fixposition_driver_lib/converter/tf.hpp>
#include <fixposition_driver_lib/converter/gpgga.hpp>
#include <fixposition_driver_lib/converter/gpzda.hpp>
#include <fixposition_driver_lib/converter/gprmc.hpp>
#include <fixposition_driver_lib/fixposition_driver.hpp>
#include <fixposition_driver_lib/helper.hpp>
#include <fixposition_gnss_tf/gnss_tf.hpp>

/* PACKAGE */
#include <fixposition_driver_ros1/data_to_ros1.hpp>
#include <fixposition_driver_ros1/fixposition_driver_node.hpp>
#include <fixposition_driver_ros1/params.hpp>

namespace fixposition {

FixpositionDriverNode::FixpositionDriverNode(const FixpositionDriverParams& params)
    : FixpositionDriver(params),
      nh_("~"),
      rawimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/rawimu", 100)),
      corrimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/corrimu", 100)),
      navsatfix_pub_(nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/navsatfix", 100)),
      navsatfix_gnss1_pub_(nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/gnss1", 100)),
      navsatfix_gnss2_pub_(nh_.advertise<sensor_msgs::NavSatFix>("/fixposition/gnss2", 100)),
      nmea_pub_(nh_.advertise<fixposition_driver_ros1::NMEA>("/fixposition/nmea", 100)),
      //   ODOMETRY
      odometry_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry", 100)),
      odometry_smooth_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odomsh", 100)),
      poiimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/poiimu", 100)),
      vrtk_pub_(nh_.advertise<fixposition_driver_ros1::VRTK>("/fixposition/vrtk", 100)),
      odometry_enu0_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_enu", 100)),
      eul_pub_(nh_.advertise<geometry_msgs::Vector3Stamped>("/fixposition/ypr", 100)),
      eul_imu_pub_(nh_.advertise<geometry_msgs::Vector3Stamped>("/fixposition/imu_ypr", 100)) {

    ws_sub_ = nh_.subscribe<fixposition_driver_ros1::Speed>(params_.customer_input.speed_topic, 100,
                                                            &FixpositionDriverNode::WsCallback, this,
                                                            ros::TransportHints().tcpNoDelay());

    RegisterObservers();
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
                    if (odometry_pub_.getNumSubscribers() > 0) {
                        nav_msgs::Odometry odometry;
                        OdometryDataToMsg(data.odometry, odometry);
                        odometry_pub_.publish(odometry);
                    }

                    if (vrtk_pub_.getNumSubscribers() > 0) {
                        fixposition_driver_ros1::VRTK vrtk;
                        VrtkDataToMsg(data.vrtk, vrtk);
                        vrtk_pub_.publish(vrtk);
                    }

                    if (poiimu_pub_.getNumSubscribers() > 0) {
                        sensor_msgs::Imu poiimu;
                        ImuDataToMsg(data.imu, poiimu);
                        poiimu_pub_.publish(poiimu);
                    }

                    if (navsatfix_pub_.getNumSubscribers() > 0) {
                        sensor_msgs::NavSatFix msg;
                        NavSatFixDataToMsg(data.odom_llh, msg);
                        navsatfix_pub_.publish(msg);
                    }

                    // TFs
                    if (data.vrtk.fusion_status > 0) {
                        geometry_msgs::TransformStamped tf_ecef_poi;
                        TfDataToMsg(data.tf_ecef_poi, tf_ecef_poi);
                        br_.sendTransform(tf_ecef_poi);
                    }
                });
        } else if (format == "ODOMENU") {
            dynamic_cast<OdomenuConverter*>(a_converters_["ODOMENU"].get())
                ->AddObserver([this](const OdomenuConverter::Msgs& data) {
                    // ODOMENU Observer Lambda
                    if (odometry_enu0_pub_.getNumSubscribers() > 0) {
                        nav_msgs::Odometry odometry_enu0;
                        OdometryDataToMsg(data.odometry, odometry_enu0);
                        odometry_enu0_pub_.publish(odometry_enu0);
                    }

                    if (eul_pub_.getNumSubscribers() > 0) {
                        geometry_msgs::Vector3Stamped ypr;
                        if (data.odometry.stamp.tow == 0.0 && data.odometry.stamp.wno == 0) {
                            ypr.header.stamp = ros::Time::now();
                        } else {
                            ypr.header.stamp = ros::Time::fromBoost(fixposition::times::GpsTimeToPtime(data.odometry.stamp));
                        }
                        ypr.header.frame_id = "FP_ENU";
                        tf::vectorEigenToMsg(data.eul, ypr.vector);
                        eul_pub_.publish(ypr);
                    }
                });
        } else if (format == "ODOMSH") {
            dynamic_cast<OdomshConverter*>(a_converters_["ODOMSH"].get())
                ->AddObserver([this](const OdomshConverter::Msgs& data) {
                    // ODOMSH Observer Lambda
                    if (odometry_smooth_pub_.getNumSubscribers() > 0) {
                        nav_msgs::Odometry odometry;
                        OdometryDataToMsg(data.odometry, odometry);
                        odometry_smooth_pub_.publish(odometry);
                    }
                });
        } else if (format == "RAWIMU") {
            dynamic_cast<ImuConverter*>(a_converters_["RAWIMU"].get())->AddObserver([this](const ImuData& data) {
                // RAWIMU Observer Lambda
                sensor_msgs::Imu msg;
                ImuDataToMsg(data, msg);
                rawimu_pub_.publish(msg);
            });
        } else if (format == "CORRIMU") {
            dynamic_cast<ImuConverter*>(a_converters_["CORRIMU"].get())->AddObserver([this](const ImuData& data) {
                // CORRIMU Observer Lambda
                sensor_msgs::Imu msg;
                ImuDataToMsg(data, msg);
                corrimu_pub_.publish(msg);
            });
        } else if (format == "TF") {
            dynamic_cast<TfConverter*>(a_converters_["TF"].get())->AddObserver([this](const TfData& data) {
                // TF Observer Lambda
                geometry_msgs::TransformStamped tf;
                TfDataToMsg(data, tf);
                if (tf.child_frame_id == "FP_IMUH" && tf.header.frame_id == "FP_POI") {
                    br_.sendTransform(tf);

                    // Publish Pitch Roll based on IMU only
                    Eigen::Vector3d imu_ypr_eigen = gnss_tf::QuatToEul(data.rotation);
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
            dynamic_cast<GpggaConverter*>(a_converters_["GPGGA"].get())->AddObserver([this](const GpggaData& data) {
                // GPGGA Observer Lambda
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.gpgga = data;
                    PublishNmea(nmea_message_);
                }
            });
        } else if (format == "GPZDA") {
            dynamic_cast<GpzdaConverter*>(a_converters_["GPZDA"].get())->AddObserver([this](const GpzdaData& data) {
                // GPZDA Observer Lambda
                if (nmea_pub_.getNumSubscribers() > 0) {
                    nmea_message_.gpzda = data;
                    PublishNmea(nmea_message_);
                }
            });
        } else if (format == "GPRMC") {
            dynamic_cast<GprmcConverter*>(a_converters_["GPRMC"].get())->AddObserver([this](const GprmcData& data) {
                // GPRMC Observer Lambda
                if (nmea_pub_.getNumSubscribers() > 0) {
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
        fixposition_driver_ros1::NMEA msg;

        // ROS Header
        if (data.gpzda.stamp.tow == 0.0 && data.gpzda.stamp.wno == 0) {
            msg.header.stamp = ros::Time::now();
        } else {
            msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.gpzda.stamp));
        }
        msg.header.frame_id = "FP_POI";

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
        nmea_pub_.publish(msg);
    }
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

void FixpositionDriverNode::WsCallback(const fixposition_driver_ros1::SpeedConstPtr& msg) {
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
