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
#include <fixposition_odometry_converter_ros1/params.hpp>
#include <fixposition_odometry_converter_ros1/ros_msgs.hpp>

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
     * @brief Converts the speed(s) to an integer value in [mm/s] and publishes it
     *
     * @param speeds vector containing x, y, and z speeds and their validity flag
     */
    void ConvertAndPublish(const std::vector<std::pair<bool, double>> speeds);

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
