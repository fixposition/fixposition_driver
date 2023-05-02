/**
 *  @file
 *  @brief Declaration of OdomConverter class
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

#ifndef __ODOM_CONVERTER_HPP__
#define __ODOM_CONVERTER_HPP__

/* EXTERNAL */

/* ROS */
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

/* PACKAGE */
#include <fixposition_driver_ros1/Speed.h>

#include <fixposition_odometry_converter/params.hpp>

namespace fixposition {

class OdomConverter {
   public:
    /**
     * @brief Construct a new OdomConverter object
     *
     * @param[in] nh node handle
     */
    OdomConverter(ros::NodeHandle* nh);

    /**
     * @brief Destroy the OdomConverter object
     *
     */
    ~OdomConverter() = default;

    /**
     * @brief Subscribes to the correct topic depending on the parameters
     *
     */
    void Subscribe();

    /**
     * @brief Converts and publishes the speed to an integer value in [mm/s]
     *
     * @param speed
     */
    void ConvertAndPublish(const double speed, const double angular, bool use_angular = false);

   private:
    /**
     * @brief Converts a message of type TwistWithCovariance to the fixposition_driver Speed message
     *
     * @param[in] msg
     */
    void TwistWithCovCallback(const geometry_msgs::TwistWithCovarianceConstPtr& msg);

    /**
     * @brief Converts a message of type Odometry to the fixposition_driver Speed message
     *
     * @param[in] msg
     */
    void OdometryCallback(const nav_msgs::OdometryConstPtr& msg);

    /**
     * @brief Converts a message of type Twist to the fixposition_driver Speed message
     *
     * @param[in] msg
     */
    void TwistCallback(const geometry_msgs::TwistConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber ws_sub_;  //!< wheelspeed message subscriber
    ros::Publisher ws_pub_;   //!< wheelspeed message publisher

    OdomInputParams params_;
};
}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_FIXPOSITION_DRIVER__
