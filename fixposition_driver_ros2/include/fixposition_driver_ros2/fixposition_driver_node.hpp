/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition driver node for ROS2
 */

#ifndef __FIXPOSITION_DRIVER_ROS2_FIXPOSITION_DRIVER_NODE_HPP__
#define __FIXPOSITION_DRIVER_ROS2_FIXPOSITION_DRIVER_NODE_HPP__

/* LIBC/STL */
#include <memory>
#include <mutex>

/* EXTERNAL */
#include <fixposition_driver_lib/helper.hpp>
#include <fpsdk_ros2/ext/rclcpp.hpp>

/* PACKAGE */
#include "data_to_ros2.hpp"
#include "params.hpp"
#include "ros2_msgs.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::common;

class FixpositionDriverNode {
   public:
    /**
     * @brief Constructor
     *
     * @param[in]  nh      Node handle
     * @param[in]  params  Parameters
     */
    FixpositionDriverNode(std::shared_ptr<rclcpp::Node> nh, const DriverParams& params);

    /**
     * @brief Destructor
     */
    ~FixpositionDriverNode();

    /**
     * @brief Start the node
     *
     * @returns true on success, false otherwise
     */
    bool StartNode();

    /**
     * @brief Stop the node
     */
    void StopNode();

   private:
    std::shared_ptr<rclcpp::Node> nh_;  //!< Node handle
    DriverParams params_;               //!< Sensor/driver parameters
    rclcpp::Logger logger_;             //!< Logger
    FixpositionDriver driver_;          //!< Sensor driver
    rclcpp::QoS qos_settings_;          //!< QoS settings

    // ROS publishers
    // - FP_A messages
    rclcpp::Publisher<fpmsgs::FpaEoe>::SharedPtr fpa_eoe_pub_;                //!< FP_A-EOE message
    rclcpp::Publisher<fpmsgs::FpaGnssant>::SharedPtr fpa_gnssant_pub_;        //!< FP_A-GNSSANT message
    rclcpp::Publisher<fpmsgs::FpaGnsscorr>::SharedPtr fpa_gnsscorr_pub_;      //!< FP_A-GNSSCORR message
    rclcpp::Publisher<fpmsgs::FpaImubias>::SharedPtr fpa_imubias_pub_;        //!< FP_A-IMUBIAS message
    rclcpp::Publisher<fpmsgs::FpaLlh>::SharedPtr fpa_llh_pub_;                //!< FP_A-LLH message
    rclcpp::Publisher<fpmsgs::FpaOdomenu>::SharedPtr fpa_odomenu_pub_;        //!< FP_A-ODOMENU message
    rclcpp::Publisher<fpmsgs::FpaOdometry>::SharedPtr fpa_odometry_pub_;      //!< FP_A-ODOMETRY message
    rclcpp::Publisher<fpmsgs::FpaOdomsh>::SharedPtr fpa_odomsh_pub_;          //!< FP_A-ODOMSH message
    rclcpp::Publisher<fpmsgs::FpaOdomstatus>::SharedPtr fpa_odomstatus_pub_;  //!< FP_A-ODOMSTATUS message
    rclcpp::Publisher<fpmsgs::FpaText>::SharedPtr fpa_text_pub_;              //!< FP_A-TEXT message
    rclcpp::Publisher<fpmsgs::FpaTp>::SharedPtr fpa_tp_pub_;                  //!< FP_A-TP message
    // - NMEA messages
    rclcpp::Publisher<fpmsgs::NmeaGga>::SharedPtr nmea_gga_pub_;  //!< NMEA-GP-GGA message
    rclcpp::Publisher<fpmsgs::NmeaGll>::SharedPtr nmea_gll_pub_;  //!< NMEA-GP-GLL message
    rclcpp::Publisher<fpmsgs::NmeaGsa>::SharedPtr nmea_gsa_pub_;  //!< NMEA-GN-GSA message
    rclcpp::Publisher<fpmsgs::NmeaGst>::SharedPtr nmea_gst_pub_;  //!< NMEA-GP-GST message
    rclcpp::Publisher<fpmsgs::NmeaGsv>::SharedPtr nmea_gsv_pub_;  //!< NMEA-GX-GSV message
    rclcpp::Publisher<fpmsgs::NmeaHdt>::SharedPtr nmea_hdt_pub_;  //!< NMEA-GP-HDT message
    rclcpp::Publisher<fpmsgs::NmeaRmc>::SharedPtr nmea_rmc_pub_;  //!< NMEA-GP-RMC message
    rclcpp::Publisher<fpmsgs::NmeaVtg>::SharedPtr nmea_vtg_pub_;  //!< NMEA-GP-VTG message
    rclcpp::Publisher<fpmsgs::NmeaZda>::SharedPtr nmea_zda_pub_;  //!< NMEA-GP-ZDA message
    // - NOV_B messages
    rclcpp::Publisher<fpmsgs::NovbInspvax>::SharedPtr novb_inspvax_pub_;  //!< NOV_B-INSPVAX message
    rclcpp::Publisher<fpmsgs::NovbHeading2>::SharedPtr novb_heading2_pub_;  //!< NOV_B-HEADING2 message
    // - Odometry
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_ecef_pub_;        //!< ECEF odometry
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_enu_pub_;         //!< ENU odometry
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_smooth_pub_;      //!< Smooth odometry (ECEF)
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_enu_smooth_pub_;  //!< Smooth odometry (ENU)
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr odometry_llh_pub_;     //!< LLH odometry
    // - Fusion
    rclcpp::Publisher<fpmsgs::FusionEpoch>::SharedPtr fusion_epoch_pub_;  //!< Fusion epoch data
    // - Orientation
    //! Euler angles yaw-pitch-roll in local ENU
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr eul_pub_;
    //! Euler angles pitch-roll as estimated from the IMU in local horizontal
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr eul_imu_pub_;
    // - IMU
    rclcpp::Publisher<fpmsgs::FpaImu>::SharedPtr rawimu_pub_;         //!< Raw IMU data in IMU frame
    rclcpp::Publisher<fpmsgs::FpaImu>::SharedPtr corrimu_pub_;        //!< Bias corrected IMU data in IMU frame
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr poiimu_pub_;  //!< Bias corrected IMU data in POI frame
    // - GNSS
    rclcpp::Publisher<fpmsgs::NmeaEpoch>::SharedPtr nmea_epoch_pub_;                 //!< NMEA epoch data
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_gnss1_pub_;  //!< GNSS1 position and status
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_gnss2_pub_;  //!< GNSS2 position and status
    // - Other
    rclcpp::Publisher<fpmsgs::CovWarn>::SharedPtr jump_pub_;               //!< Jump warning topic
    rclcpp::Publisher<fpmsgs::ParserMsg>::SharedPtr raw_pub_;              //!< Raw messages topic
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr datum_pub_;  //!< WGS84 datum topic

    // ROS subscribers
    rclcpp::Subscription<fpmsgs::Speed>::SharedPtr ws_sub_;              //!< Wheelspeed input subscriber
    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr corr_sub_;  //!< GNSS correction data input subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        ws_conv_twist_sub_;  //!< Wheelspeed input converter subscriber (Twist)
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovariance>::SharedPtr
        ws_conv_twistcov_sub_;  //!< Wheelspeed input converter subscriber (TwistWithCovariance)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        ws_conv_odom_sub_;  //!< Wheelspeed input converter subscriber (Odometry)

    // TF broadcasters
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;

    // State
    JumpDetector jump_detector_;
    NmeaEpochData nmea_epoch_data_;      //!< NMEA epoch data collector
    FusionEpochData fusion_epoch_data_;  //!< Fusion epoch data collector

    // TFs
    struct Tfs {
        std::mutex mutex_;
        std::unique_ptr<geometry_msgs::msg::TransformStamped> ecef_enu0_;
        std::unique_ptr<geometry_msgs::msg::TransformStamped> poi_poish_;
        std::unique_ptr<geometry_msgs::msg::TransformStamped> ecef_poish_;
        std::unique_ptr<geometry_msgs::msg::TransformStamped> enu0_poi_;
    };
    Tfs tfs_;
    std::unique_ptr<TfData> ecef_enu0_tf_;

    void ProcessTfData(const TfData& tf_data);
    void ProcessOdometryData(const OdometryData& odometry_data);
    void PublishNav2Tf();
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS2_FIXPOSITION_DRIVER_NODE_HPP__
