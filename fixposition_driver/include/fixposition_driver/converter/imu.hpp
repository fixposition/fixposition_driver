/**
 *  @file
 *  @brief Declaration of ImuConverter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */
#ifndef __FIXPOSITION_DRIVER_CONVERTER_IMU__
#define __FIXPOSITION_DRIVER_CONVERTER_IMU__

/* ROS */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

/* PACKAGE */
#include <fixposition_driver/converter/base_converter.hpp>
#include <fixposition_driver/time_conversions.hpp>

namespace fixposition {

class ImuConverter : public BaseConverter {
   public:
    /**
     * @brief Construct a new ImuConverter
     *
     */
    ImuConverter(std::shared_ptr<rclcpp::Node> node, const bool bias_correction)
        : BaseConverter(node)
	, imu_pub_(node->create_publisher<sensor_msgs::msg::Imu>(
	    (bias_correction ? "/fixposition/corrimu" : "/fixposition/rawimu"), 100))
        , header_(bias_correction ? "CORRIMU" : "RAWIMU")
        , bias_correction_(bias_correction) {}

    ~ImuConverter() = default;

    bool CheckHeaderAndVersion(const std::string msg_header, const std::string msg_version) final {
        return msg_header == header_ && std::stoi(msg_version) == kVersion_;
    }

    /**
     * @brief Take comma-delimited tokens of FP,RAWIMU or FP,RAWIMU message and publish them as ros message
     * Examples:
     * $FP,RAWIMU,1,2197,126191.777855,-0.199914,0.472851,9.917973,0.023436,0.007723,0.002131*34
     * $FP,CORRIMU,1,2197,126191.777855,-0.195224,0.393969,9.869998,0.013342,-0.004620,-0.000728*7D
     *
     * @param[in] tokens message split in tokens
     */
    void ConvertTokensAndPublish(const std::vector<std::string>& tokens) final;

   private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    const std::string header_;

    const bool bias_correction_;
    static constexpr const int kVersion_ = 1;
};
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_CONVERTER_IMU__
