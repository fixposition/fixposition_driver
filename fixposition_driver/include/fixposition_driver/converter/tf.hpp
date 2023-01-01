/**
 *  @file
 *  @brief Declaration of TfConverter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_CONVERTER_TF__
#define __FIXPOSITION_DRIVER_CONVERTER_TF__


/* ROS */
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

/* PACKAGE */
#include <fixposition_driver/converter/base_converter.hpp>

namespace fixposition {

class TfConverter : public BaseConverter {
   public:
    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
     TfConverter(std::shared_ptr<rclcpp::Node> node) : BaseConverter(), node_(node)
	{ 
	  br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
	  static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
	}

    ~TfConverter() = default;

    bool CheckHeaderAndVersion(const std::string msg_header, const std::string msg_version) final {
        return msg_header == header_ && std::stoi(msg_version) == kVersion_;
    }
    /**
     * @brief TODO
     *
     * @param[in] state state message as string
     * @return nav_msgs::Odometry message
     */
    void ConvertTokensAndPublish(const std::vector<std::string>& tokens) final;

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
    const std::string header_ = "TF";

    static constexpr const int kVersion_ = 1;
};
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_CONVERTER_TF__
