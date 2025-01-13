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
 * @brief Declaration of OdomConverterNode class
 */

#ifndef __FIXPOSITION_ODOMETRY_CONVERTER_ROS2_ODOM_CONVERTER_HPP__
#define __FIXPOSITION_ODOMETRY_CONVERTER_ROS2_ODOM_CONVERTER_HPP__

/* LIBC/STL */

/* EXTERNAL */

/* PACKAGE */
#include "fixposition_odometry_converter_ros2/params.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

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
     * @brief Converts the speed(s) to an integer value in [mm/s] and publishes it
     *
     * @param speeds vector containing x, y, and z speeds and their validity flag
     */
    void ConvertAndPublish(const std::vector<std::pair<bool, double>> speeds);

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
    rclcpp::Publisher<fixposition_driver_msgs::msg::Speed>::SharedPtr ws_pub_;
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  //__FIXPOSITION_ODOMETRY_CONVERTER_ROS2_ODOM_CONVERTER_HPP__
