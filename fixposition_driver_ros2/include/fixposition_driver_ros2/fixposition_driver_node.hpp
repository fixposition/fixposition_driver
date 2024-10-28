/**
 *  @file
 *  @brief Declaration of FixpositionDriver ROS2 Node
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

#ifndef __FIXPOSITION_DRIVER_ROS2_FIXPOSITION_DRIVER_NODE_
#define __FIXPOSITION_DRIVER_ROS2_FIXPOSITION_DRIVER_NODE_

/* SYSTEM / STL */
#include <termios.h>

/* ROS2 */
#include <fixposition_driver_ros2/ros2_msgs.hpp>
#include <fixposition_driver_ros2/params.hpp>

/* FIXPOSITION */
#include <fixposition_driver_lib/helper.hpp>
#include <fixposition_driver_lib/gnss_tf.hpp>

/* PACKAGE */
#include <fixposition_driver_ros2/data_to_ros2.hpp>

namespace fixposition {
class FixpositionDriverNode : public FixpositionDriver {
   public:
    /**
     * @brief Construct a new Fixposition Driver Node object
     *
     * @param[in] params
     */
    FixpositionDriverNode(std::shared_ptr<rclcpp::Node> node, const FixpositionDriverParams& params, rclcpp::QoS qos_settings);

    void Run();

    void RegisterObservers();

    void WsCallbackRos(const fixposition_driver_ros2::msg::Speed::ConstSharedPtr msg);

    void RtcmCallbackRos(const rtcm_msgs::msg::Message::ConstSharedPtr msg);

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
    std::shared_ptr<rclcpp::Node> node_;

    // ROS subscribers
    rclcpp::Subscription<fixposition_driver_ros2::msg::Speed>::SharedPtr ws_sub_;  //!< wheelspeed message subscriber
    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;            //!< RTCM3 message subscriber

    // ROS publishers
    // FP_A messages
    rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMETRY>::SharedPtr fpa_odometry_pub_;      //!< FP_A-ODOMETRY message
    rclcpp::Publisher<fixposition_driver_ros2::msg::IMUBIAS>::SharedPtr fpa_imubias_pub_;        //!< FP_A-IMUBIAS message
    rclcpp::Publisher<fixposition_driver_ros2::msg::EOE>::SharedPtr fpa_eoe_pub_;                //!< FP_A-EOE message
    rclcpp::Publisher<fixposition_driver_ros2::msg::LLH>::SharedPtr fpa_llh_pub_;                //!< FP_A-LLH message
    rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMENU>::SharedPtr fpa_odomenu_pub_;        //!< FP_A-ODOMENU message
    rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMSH>::SharedPtr fpa_odomsh_pub_;          //!< FP_A-ODOMSH message
    rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMSTATUS>::SharedPtr fpa_odomstatus_pub_;  //!< FP_A-ODOMSTATUS message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GNSSANT>::SharedPtr fpa_gnssant_pub_;        //!< FP_A-GNSSANT message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GNSSCORR>::SharedPtr fpa_gnsscorr_pub_;      //!< FP_A-GNSSCORR message
    rclcpp::Publisher<fixposition_driver_ros2::msg::TEXT>::SharedPtr fpa_text_pub_;              //!< FP_A-TEXT message
    rclcpp::Publisher<fixposition_driver_ros2::msg::TP>::SharedPtr fpa_tp_pub_;                  //!< FP_A-TP message

    // NMEA messages
    rclcpp::Publisher<fixposition_driver_ros2::msg::GPGGA>::SharedPtr nmea_gpgga_pub_;           //!< NMEA-GP-GGA message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GPGLL>::SharedPtr nmea_gpgll_pub_;           //!< NMEA-GP-GLL message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GNGSA>::SharedPtr nmea_gngsa_pub_;           //!< NMEA-GP-GSA message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GPGST>::SharedPtr nmea_gpgst_pub_;           //!< NMEA-GP-GST message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GXGSV>::SharedPtr nmea_gxgsv_pub_;           //!< NMEA-GP-GSV message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GPHDT>::SharedPtr nmea_gphdt_pub_;           //!< NMEA-GP-HDT message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GPRMC>::SharedPtr nmea_gprmc_pub_;           //!< NMEA-GP-RMC message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GPVTG>::SharedPtr nmea_gpvtg_pub_;           //!< NMEA-GP-VTG message
    rclcpp::Publisher<fixposition_driver_ros2::msg::GPZDA>::SharedPtr nmea_gpzda_pub_;           //!< NMEA-GP-ZDA message

    // ODOMETRY
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_ecef_pub_;    //!< ECEF Odometry
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr odometry_llh_pub_; //!< LLH Odometry
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_enu_pub_;     //!< ENU Odometry
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_smooth_pub_;  //!< Smooth Odometry (ECEF)

    // Orientation
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr eul_pub_;              //!< Euler angles Yaw-Pitch-Roll in local ENU
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr eul_imu_pub_;          //!< Euler angles Pitch-Roll as estimated from the IMU in local horizontal

    // IMU
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr rawimu_pub_;                        //!< Raw IMU data in IMU frame
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrimu_pub_;                       //!< Bias corrected IMU data in IMU frame
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr poiimu_pub_;                        //!< Bias corrected IMU data in POI frame

    // GNSS
    rclcpp::Publisher<fixposition_driver_ros2::msg::NMEA>::SharedPtr nmea_pub_;             //!< Pose estimation based only on GNSS
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_gnss1_pub_;         //!< GNSS1 position and status
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_gnss2_pub_;         //!< GNSS2 position and status
    NmeaMessage nmea_message_;                                                              //!< Collector class for NMEA messages
    
    // TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;

    // Jump warning topic
    rclcpp::Publisher<fixposition_driver_ros2::msg::COVWARN>::SharedPtr extras_jump_pub_;   //!< Jump warning topic

    // Previous state
    Eigen::Vector3d prev_pos;
    Eigen::MatrixXd prev_cov;

    // Nav2 TF map
    std::map<std::string, std::shared_ptr<geometry_msgs::msg::TransformStamped>> tf_map = {
        {"ECEFENU0", nullptr},
        {"POIPOISH", nullptr},
        {"ECEFPOISH", nullptr},
        {"ENU0POI", nullptr}
    };
};

}  // namespace fixposition

#endif
