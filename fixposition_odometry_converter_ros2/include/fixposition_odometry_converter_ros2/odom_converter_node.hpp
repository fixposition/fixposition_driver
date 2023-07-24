/**
 *  @file
 *  @brief Declaration of OdomConverterNode class
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 * Port to ROS 2 by Husarion
 * \endverbatim
 *
 */

#ifndef __ODOM_CONVERTER_HPP__
#define __ODOM_CONVERTER_HPP__

/* ROS */
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

/* PACKAGE */
#include <fixposition_driver_ros2/msg/speed.hpp>
#include <fixposition_odometry_converter_ros2/params.hpp>

namespace fixposition {

class OdomConverterNode : public rclcpp::Node {
   public:
    /**
     * @brief Construct a new OdomConverterNode object
     *
     * @param[in] options node options
     */
    OdomConverterNode(const rclcpp::NodeOptions& options);

    /**
     * @brief Destroy the OdomConverterNode object
     *
     */
    ~OdomConverterNode() = default;

    /**
     * @brief Subscribes to the correct topic depending on the parameters
     *
     */
    void Subscribe();

    /**
     * @brief Converts and publishes the speed to an integer value in [mm/s]
     *
     * @param speed
     * @param angular
     * @param use_angular default false
     */
    void ConvertAndPublish(const double speed, const double angular, bool use_angular = false);

   private:
    /**
     * @brief Converts a message of type TwistWithCovariance to the fixposition_driver Speed message and publishes it
     *
     * @param[in] msg
     */
    void TwistWithCovCallback(const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg);

    /**
     * @brief Converts a message of type Odometry to the fixposition_driver Speed message and publishes it
     *
     * @param[in] msg
     */
    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Converts a message of type Twist to the fixposition_driver Speed message and publishes it
     *
     * @param[in] msg
     */
    void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    OdomInputParams params_;
    rclcpp::SubscriptionBase::SharedPtr ws_sub_;
    rclcpp::Publisher<fixposition_driver_ros2::msg::Speed>::SharedPtr ws_pub_;
};
}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_FIXPOSITION_DRIVER__
