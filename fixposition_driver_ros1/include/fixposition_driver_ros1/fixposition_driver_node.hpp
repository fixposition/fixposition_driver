/**
 *  @file
 *  @brief Declaration of FixpositionDriver ROS1 Node
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

#ifndef __FIXPOSITION_DRIVER_ROS1_FIXPOSITION_DRIVER_NODE_
#define __FIXPOSITION_DRIVER_ROS1_FIXPOSITION_DRIVER_NODE_

/* SYSTEM / STL */
#include <termios.h>

/* ROS */
#include <fixposition_driver_ros1/ros_msgs.hpp>
#include <fixposition_driver_ros1/params.hpp>

/* FIXPOSITION */
#include <fixposition_driver_lib/helper.hpp>
#include <fixposition_driver_lib/gnss_tf.hpp>

/* PACKAGE */
#include <fixposition_driver_ros1/data_to_ros1.hpp>

namespace fixposition {
class FixpositionDriverNode : public FixpositionDriver {
   public:
    /**
     * @brief Construct a new Fixposition Driver Node object
     *
     * @param[in] params
     */
    FixpositionDriverNode(const FixpositionDriverParams& params);

    void Run();

    void RegisterObservers();

    void WsCallbackRos(const fixposition_driver_ros1::SpeedConstPtr& msg);

    void RtcmCallbackRos(const rtcm_msgs::MessageConstPtr& msg);

   private:
    /**
     * @brief Observer Functions to publish NavSatFix from BestGnssPos
     *
     * @param[in] header
     * @param[in] payload
     */
    void BestGnssPosToPublishNavSatFix(const Oem7MessageHeaderMem* header, const BESTGNSSPOSMem* payload);

    /**
     * @brief Observer Function to publish NMEA message once the GNSS epoch transmission is complete
     *
     * @param[in] data
     */
    void PublishNmea();

    // ROS node handler
    ros::NodeHandle nh_;
    
    // ROS subscribers
    ros::Subscriber ws_sub_;  //!< wheelspeed message subscriber
    ros::Subscriber rtcm_sub_;  //!< RTCM3 message subscriber

    // ROS publishers
    // FP_A messages
    ros::Publisher fpa_odometry_pub_;    //!< FP_A-ODOMETRY message
    ros::Publisher fpa_imubias_pub_;     //!< FP_A-IMUBIAS message
    ros::Publisher fpa_eoe_pub_;         //!< FP_A-EOE message
    ros::Publisher fpa_llh_pub_;         //!< FP_A-LLH message
    ros::Publisher fpa_odomenu_pub_;     //!< FP_A-ODOMENU message
    ros::Publisher fpa_odomsh_pub_;      //!< FP_A-ODOMSH message
    ros::Publisher fpa_odomstatus_pub_;  //!< FP_A-ODOMSTATUS message
    ros::Publisher fpa_gnssant_pub_;     //!< FP_A-GNSSANT message
    ros::Publisher fpa_gnsscorr_pub_;    //!< FP_A-GNSSCORR message
    ros::Publisher fpa_text_pub_;        //!< FP_A-TEXT message
    ros::Publisher fpa_tp_pub_;          //!< FP_A-TP message

    // NMEA messages
    ros::Publisher nmea_gpgga_pub_;      //!< NMEA-GP-GGA message
    ros::Publisher nmea_gpgll_pub_;      //!< NMEA-GP-GLL message
    ros::Publisher nmea_gngsa_pub_;      //!< NMEA-GP-GSA message
    ros::Publisher nmea_gpgst_pub_;      //!< NMEA-GP-GST message
    ros::Publisher nmea_gxgsv_pub_;      //!< NMEA-GP-GSV message
    ros::Publisher nmea_gphdt_pub_;      //!< NMEA-GP-HDT message
    ros::Publisher nmea_gprmc_pub_;      //!< NMEA-GP-RMC message
    ros::Publisher nmea_gpvtg_pub_;      //!< NMEA-GP-VTG message
    ros::Publisher nmea_gpzda_pub_;      //!< NMEA-GP-ZDA message

    // ODOMETRY
    ros::Publisher odometry_ecef_pub_;   //!< ECEF Odometry
    ros::Publisher odometry_llh_pub_;    //!< LLH Odometry
    ros::Publisher odometry_enu_pub_;    //!< ENU Odometry
    ros::Publisher odometry_smooth_pub_; //!< Smooth Odometry (ECEF)
    
    // Orientation
    ros::Publisher eul_pub_;             //!< Euler angles Yaw-Pitch-Roll in local ENU
    ros::Publisher eul_imu_pub_;         //!< Euler angles Pitch-Roll as estimated from the IMU in local horizontal

    // IMU
    ros::Publisher rawimu_pub_;          //!< Raw IMU data in IMU frame
    ros::Publisher corrimu_pub_;         //!< Bias corrected IMU data in IMU frame
    ros::Publisher poiimu_pub_;          //!< Bias corrected IMU data in POI frame

    // GNSS
    ros::Publisher nmea_pub_;            //!< Pose estimation based only on GNSS
    ros::Publisher navsatfix_gnss1_pub_; //!< GNSS1 position and status
    ros::Publisher navsatfix_gnss2_pub_; //!< GNSS2 position and status
    NmeaMessage nmea_message_;           //!< Collector class for NMEA messages
    
    // TF
    tf2_ros::TransformBroadcaster br_;
    tf2_ros::StaticTransformBroadcaster static_br_;

    // Jump warning topic
    ros::Publisher extras_jump_pub_;     //!< Jump warning topic

    // Previous state
    Eigen::Vector3d prev_pos;
    Eigen::MatrixXd prev_cov;

    // Nav2 TF map
    std::map<std::string, std::shared_ptr<geometry_msgs::TransformStamped>> tf_map = {
        {"ECEFENU0", nullptr},
        {"POIPOISH", nullptr},
        {"ECEFPOISH", nullptr},
        {"ENU0POI", nullptr}
    };
};

}  // namespace fixposition

#endif
