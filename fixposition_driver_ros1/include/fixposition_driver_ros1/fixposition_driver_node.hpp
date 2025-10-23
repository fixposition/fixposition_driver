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
 * @brief Fixposition driver node for ROS1
 */

#ifndef __FIXPOSITION_DRIVER_ROS1_FIXPOSITION_DRIVER_NODE_HPP__
#define __FIXPOSITION_DRIVER_ROS1_FIXPOSITION_DRIVER_NODE_HPP__

/* LIBC/STL */
#include <memory>
#include <mutex>

/* EXTERNAL */
#include <fixposition_driver_lib/helper.hpp>
#include <fpsdk_ros1/ext/ros.hpp>

/* PACKAGE */
#include "data_to_ros1.hpp"
#include "params.hpp"
#include "ros1_msgs.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

class FixpositionDriverNode {
   public:
    /**
     * @brief Constructor
     *
     * @param[in]  params  Parameters
     * @param[in]  nh      Node handle
     */
    FixpositionDriverNode(const DriverParams& params, ros::NodeHandle& nh);

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
    ros::NodeHandle nh_;        //!< ROS node handle
    DriverParams params_;       //!< Sensor/driver parameters
    FixpositionDriver driver_;  //!< Sensor driver

    // ROS publishers
    // - FP_A messages
    ros::Publisher fpa_eoe_pub_;         //!< FP_A-EOE message
    ros::Publisher fpa_gnssant_pub_;     //!< FP_A-GNSSANT message
    ros::Publisher fpa_gnsscorr_pub_;    //!< FP_A-GNSSCORR message
    ros::Publisher fpa_imubias_pub_;     //!< FP_A-IMUBIAS message
    ros::Publisher fpa_llh_pub_;         //!< FP_A-LLH message
    ros::Publisher fpa_odomenu_pub_;     //!< FP_A-ODOMENU message
    ros::Publisher fpa_odometry_pub_;    //!< FP_A-ODOMETRY message
    ros::Publisher fpa_odomsh_pub_;      //!< FP_A-ODOMSH message
    ros::Publisher fpa_odomstatus_pub_;  //!< FP_A-ODOMSTATUS message
    ros::Publisher fpa_text_pub_;        //!< FP_A-TEXT message
    ros::Publisher fpa_tp_pub_;          //!< FP_A-TP message
    // - NMEA messages
    ros::Publisher nmea_gga_pub_;  //!< NMEA-GP-GGA message
    ros::Publisher nmea_gll_pub_;  //!< NMEA-GP-GLL message
    ros::Publisher nmea_gsa_pub_;  //!< NMEA-GN-GSA message
    ros::Publisher nmea_gst_pub_;  //!< NMEA-GP-GST message
    ros::Publisher nmea_gsv_pub_;  //!< NMEA-GX-GSV message
    ros::Publisher nmea_hdt_pub_;  //!< NMEA-GP-HDT message
    ros::Publisher nmea_rmc_pub_;  //!< NMEA-GP-RMC message
    ros::Publisher nmea_vtg_pub_;  //!< NMEA-GP-VTG message
    ros::Publisher nmea_zda_pub_;  //!< NMEA-GP-ZDA message
    // - NOV_B messages
    ros::Publisher novb_inspvax_pub_;   //!< NOV_B-INSPVAX message
    ros::Publisher novb_heading2_pub_;  //!< NOV_B-HEADING2 message
    // - Odometry
    ros::Publisher odometry_ecef_pub_;        //!< ECEF odometry
    ros::Publisher odometry_enu_pub_;         //!< ENU odometry
    ros::Publisher odometry_smooth_pub_;      //!< Smooth Odometry (ECEF)
    ros::Publisher odometry_enu_smooth_pub_;  //!< Smooth Odometry (ENU)
    ros::Publisher odometry_llh_pub_;         //!< LLH odometry
    // - Fusion
    ros::Publisher fusion_epoch_pub_;  //!< Fusion epoch data
    // - Orientation
    ros::Publisher eul_pub_;      //!< Euler angles yaw-pitch-roll in local ENU
    ros::Publisher eul_imu_pub_;  //!< Euler angles pitch-roll as estimated from the IMU in local horizontal
    // - IMU
    ros::Publisher corrimu_pub_;  //!< Bias corrected IMU data in IMU frame
    ros::Publisher poiimu_pub_;   //!< Bias corrected IMU data in POI frame
    ros::Publisher rawimu_pub_;   //!< Raw IMU data in IMU frame
    // - GNSS
    ros::Publisher navsatfix_gnss1_pub_;  //!< GNSS1 position and status
    ros::Publisher navsatfix_gnss2_pub_;  //!< GNSS2 position and status
    ros::Publisher nmea_epoch_pub_;       //!< NMEA epoch data
    // - Other
    ros::Publisher jump_pub_;   //!< Jump warning topic
    ros::Publisher raw_pub_;    //!< Raw messages topic
    ros::Publisher datum_pub_;  //!< WGS84 datum topic

    // ROS subscribers
    ros::Subscriber ws_sub_;       //!< Wheelspeed input subscriber
    ros::Subscriber corr_sub_;     //!< GNSS correction data input subscriber
    ros::Subscriber ws_conv_sub_;  //!< Wheelspeed input converter subscriber

    // TF2 broadcasters
    tf2_ros::TransformBroadcaster tf_br_;
    tf2_ros::StaticTransformBroadcaster static_br_;

    // State
    JumpDetector jump_detector_;
    NmeaEpochData nmea_epoch_data_;      //!< NMEA epoch data collector
    FusionEpochData fusion_epoch_data_;  //!< Fusion epoch data collector

    // TFs
    struct Tfs {
        std::mutex mutex_;
        std::unique_ptr<geometry_msgs::TransformStamped> ecef_enu0_;
        std::unique_ptr<geometry_msgs::TransformStamped> poi_poish_;
        std::unique_ptr<geometry_msgs::TransformStamped> ecef_poish_;
        std::unique_ptr<geometry_msgs::TransformStamped> enu0_poi_;
    };
    Tfs tfs_;
    std::unique_ptr<TfData> ecef_enu0_tf_;

    void ProcessTfData(const TfData& tf_data);
    void ProcessOdometryData(const OdometryData& odometry_data);
    void PublishNav2Tf();
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS1_FIXPOSITION_DRIVER_NODE_HPP__
