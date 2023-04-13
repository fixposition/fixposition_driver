/**
 *  @file
 *  @brief Main function for the fixposition driver ros node
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* ROS */
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <ros/ros.h>

/* FIXPOSITION */
#include <fixposition_driver_lib/converter/imu.hpp>
#include <fixposition_driver_lib/converter/llh.hpp>
#include <fixposition_driver_lib/converter/odometry.hpp>
#include <fixposition_driver_lib/converter/tf.hpp>
#include <fixposition_driver_lib/fixposition_driver.hpp>
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
      //   ODOMETRY
      odometry_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry", 100)),
      poiimu_pub_(nh_.advertise<sensor_msgs::Imu>("/fixposition/poiimu", 100)),
      vrtk_pub_(nh_.advertise<fixposition_driver_ros1::VRTK>("/fixposition/vrtk", 100)),
      odometry_enu0_pub_(nh_.advertise<nav_msgs::Odometry>("/fixposition/odometry_enu", 100)),
      eul_pub_(nh_.advertise<geometry_msgs::Vector3>("/fixposition/ypr", 100)),
      eul_imu_pub_(nh_.advertise<geometry_msgs::Vector3>("/fixposition/imu_ypr", 100)) {
    ws_sub_ = nh_.subscribe<fixposition_driver_ros1::Speed>(params_.customer_input.speed_topic, 100,
                                                            &FixpositionDriverNode::WsCallback, this,
                                                            ros::TransportHints().tcpNoDelay());
    Connect();
    RegisterObservers();
}

void FixpositionDriverNode::RegisterObservers() {
    for (const auto& format : params_.fp_output.formats) {
        if (format == "ODOMETRY") {
            dynamic_cast<OdometryConverter*>(converters_["ODOMETRY"].get())
                ->AddObserver([this](const OdometryConverter::Msgs& data) {
                    // ODOMETRY Observer Lambda
                    // Msgs
                    if (odometry_pub_.getNumSubscribers() > 0) {
                        nav_msgs::Odometry odometry;
                        OdometryDataToMsg(data.odometry, odometry);
                        odometry_pub_.publish(odometry);
                    }

                    if (odometry_enu0_pub_.getNumSubscribers() > 0) {
                        nav_msgs::Odometry odometry_enu0;
                        OdometryDataToMsg(data.odometry_enu0, odometry_enu0);
                        odometry_enu0_pub_.publish(odometry_enu0);
                    }

                    if (vrtk_pub_.getNumSubscribers() > 0) {
                        fixposition_driver_ros1::VRTK vrtk;
                        VrtkDataToMsg(data.vrtk, vrtk);
                        vrtk_pub_.publish(vrtk);
                    }
                    if (eul_pub_.getNumSubscribers() > 0) {
                        geometry_msgs::Vector3 ypr;
                        tf::vectorEigenToMsg(data.eul, ypr);
                        eul_pub_.publish(ypr);
                    }

                    if (poiimu_pub_.getNumSubscribers() > 0) {
                        sensor_msgs::Imu poiimu;
                        ImuDataToMsg(data.imu, poiimu);
                        poiimu_pub_.publish(poiimu);
                    }

                    // TFs
                    if (data.vrtk.fusion_status > 0) {
                        geometry_msgs::TransformStamped tf_ecef_poi;
                        geometry_msgs::TransformStamped tf_ecef_enu;
                        geometry_msgs::TransformStamped tf_ecef_enu0;
                        TfDataToMsg(data.tf_ecef_poi, tf_ecef_poi);
                        TfDataToMsg(data.tf_ecef_enu, tf_ecef_enu);
                        TfDataToMsg(data.tf_ecef_enu0, tf_ecef_enu0);

                        br_.sendTransform(tf_ecef_enu);
                        br_.sendTransform(tf_ecef_poi);
                        static_br_.sendTransform(tf_ecef_enu0);
                    }
                });
        } else if (format == "LLH" && converters_["LLH"]) {
            dynamic_cast<LlhConverter*>(converters_["LLH"].get())->AddObserver([this](const NavSatFixData& data) {
                // LLH Observer Lambda
                sensor_msgs::NavSatFix msg;
                NavSatFixDataToMsg(data, msg);
                navsatfix_pub_.publish(msg);
            });
        } else if (format == "RAWIMU") {
            dynamic_cast<ImuConverter*>(converters_["RAWIMU"].get())->AddObserver([this](const ImuData& data) {
                // RAWIMU Observer Lambda
                sensor_msgs::Imu msg;
                ImuDataToMsg(data, msg);
                rawimu_pub_.publish(msg);
            });
        } else if (format == "CORRIMU") {
            dynamic_cast<ImuConverter*>(converters_["CORRIMU"].get())->AddObserver([this](const ImuData& data) {
                // CORRIMU Observer Lambda
                sensor_msgs::Imu msg;
                ImuDataToMsg(data, msg);
                corrimu_pub_.publish(msg);
            });
        } else if (format == "TF") {
            dynamic_cast<TfConverter*>(converters_["TF"].get())->AddObserver([this](const TfData& data) {
                // TF Observer Lambda
                geometry_msgs::TransformStamped tf;
                TfDataToMsg(data, tf);
                if (tf.child_frame_id == "FP_IMU_HORIZONTAL" && tf.header.frame_id == "FP_POI") {
                    br_.sendTransform(tf);

                    // Publish Pitch Roll based on IMU only
                    Eigen::Vector3d imu_ypr_eigen = gnss_tf::QuatToEul(data.rotation);
                    imu_ypr_eigen.x() = 0.0;  // the yaw value is not observable using IMU alone
                    geometry_msgs::Vector3 imu_ypr;
                    tf::vectorEigenToMsg(imu_ypr_eigen, imu_ypr);
                    eul_imu_pub_.publish(imu_ypr);

                } else {
                    static_br_.sendTransform(tf);
                }
            });
        }
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
            rate.sleep(); //ensure the loop runs at the desired rate
        }
}

void FixpositionDriverNode::WsCallback(const fixposition_driver_ros1::SpeedConstPtr& msg) {
    // ROS_WARN("WsCallback");
    FixpositionDriver::WsCallback(msg->speeds);
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
