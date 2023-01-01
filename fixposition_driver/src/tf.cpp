/**
 *  @file
 *  @brief Implementation of TfConverter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* ROS */
#include <geometry_msgs/msg/transform_stamped.hpp>

/* PACKAGE */
#include <fixposition_driver/converter/tf.hpp>

namespace fixposition {

/// msg field indices
static constexpr const int msg_type_idx = 1;
static constexpr const int msg_version_idx = 2;
static constexpr const int from_frame_idx = 3;
static constexpr const int to_frame_idx = 4;
static constexpr const int translation_x_idx = 5;
static constexpr const int translation_y_idx = 6;
static constexpr const int translation_z_idx = 7;
static constexpr const int orientation_w_idx = 8;
static constexpr const int orientation_x_idx = 9;
static constexpr const int orientation_y_idx = 10;
static constexpr const int orientation_z_idx = 11;

void TfConverter::ConvertTokensAndPublish(const std::vector<std::string>& tokens) {
    geometry_msgs::msg::TransformStamped tf;
    if (tokens.size() != 12) {
        RCLCPP_INFO(node_->get_logger(), "Error in parsing TF string with %lu fields! TFs will be empty.", tokens.size());
        return;
    }
    // header stamps
    tf.header.stamp = rclcpp::Clock().now();
    tf.header.frame_id = "FP_" + tokens.at(from_frame_idx);
    tf.child_frame_id = "FP_" + tokens.at(to_frame_idx);

    tf.transform.translation.x = StringToDouble(tokens.at(translation_x_idx));
    tf.transform.translation.y = StringToDouble(tokens.at(translation_y_idx));
    tf.transform.translation.z = StringToDouble(tokens.at(translation_z_idx));

    tf.transform.rotation.w = StringToDouble(tokens.at(orientation_w_idx));
    tf.transform.rotation.x = StringToDouble(tokens.at(orientation_x_idx));
    tf.transform.rotation.y = StringToDouble(tokens.at(orientation_y_idx));
    tf.transform.rotation.z = StringToDouble(tokens.at(orientation_z_idx));

    if (CheckQuat(tf.transform.rotation)) {
        if (tf.child_frame_id == "FP_IMU_HORIZONTAL") {
            br_->sendTransform(tf);
        } else {
            static_br_->sendTransform(tf);
        }
    }
}

}  // namespace fixposition
