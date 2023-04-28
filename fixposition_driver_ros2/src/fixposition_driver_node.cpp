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
      odometry_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry", 100)),
      poiimu_pub_(node_->create_publisher<sensor_msgs::msg::Imu>("/fixposition/poiimu", 100)),
      vrtk_pub_(node_->create_publisher<fixposition_driver_ros2::msg::VRTK>("/fixposition/vrtk", 100)),
      odometry_enu0_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry_enu", 100)),
      eul_pub_(node_->create_publisher<geometry_msgs::msg::Vector3>("/fixposition/ypr", 100)),
      eul_imu_pub_(node_->create_publisher<geometry_msgs::msg::Vector3>("/fixposition/imu_ypr", 100)),
      br_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
      static_br_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_)) {
    ws_sub_ = node_->create_subscription<fixposition_driver_ros2::msg::Speed>(
        params_.customer_input.speed_topic, 100,
        std::bind(&FixpositionDriverNode::WsCallback, this, std::placeholders::_1));

    Connect();
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
    bestgnsspos_obs_.push_back([this](const Oem7MessageHeaderMem* header, const BESTGNSSPOSMem* payload) {
        NavSatFixData nav_sat_fix;
        NovToData(header, payload, nav_sat_fix);

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
    });

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
                        geometry_msgs::msg::Vector3 ypr;
                        ypr.set__x(data.eul.x());
                        ypr.set__y(data.eul.y());
                        ypr.set__z(data.eul.z());
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
                if (tf.child_frame_id == "FP_IMU_HORIZONTAL" && tf.header.frame_id == "FP_POI") {
                    br_->sendTransform(tf);

                    // Publish Pitch Roll based on IMU only
                    Eigen::Vector3d imu_ypr_eigen = gnss_tf::QuatToEul(data.rotation);
                    imu_ypr_eigen.x() = 0.0;  // the yaw value is not observable using IMU alone
                    geometry_msgs::msg::Vector3 imu_ypr;
                    imu_ypr.set__x(imu_ypr_eigen.x());
                    imu_ypr.set__y(imu_ypr_eigen.y());
                    imu_ypr.set__z(imu_ypr_eigen.z());
                    eul_imu_pub_->publish(imu_ypr);

                } else {
                    static_br_->sendTransform(tf);
                }
            });
        }
    }
}

void FixpositionDriverNode::WsCallback(const fixposition_driver_ros2::msg::Speed::ConstSharedPtr msg) {
    FixpositionDriver::WsCallback(msg->speeds);
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
