/**
 *  @file
 *  @brief Declaration of LlhConverter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_CONVERTER_LLH__
#define __FIXPOSITION_DRIVER_CONVERTER_LLH__

/* SYSTEM / STL */

/* EXTERNAL */

/* ROS */
#include <sensor_msgs/msg/nav_sat_fix.hpp>

/* PACKAGE */
#include <fixposition_driver/msg/vrtk.hpp>

#include <fixposition_driver/converter/base_converter.hpp>
#include <fixposition_driver/time_conversions.hpp>

namespace fixposition {

class LlhConverter : public BaseConverter {
   public:
    /**
     * @brief Construct a new LlhConverter
     *
     */
    LlhConverter(std::shared_ptr<rclcpp::Node> node) : node_(node)
        , navsatfix_pub_(node->create_publisher<sensor_msgs::msg::NavSatFix>("/fixposition/navsatfix", 100)) {}

    ~LlhConverter() = default;

    bool CheckHeaderAndVersion(const std::string msg_header, const std::string msg_version) final {
        return msg_header == header_ && std::stoi(msg_version) == kVersion_;
    }
    /**
     * @brief Take comma-delimited tokens of FP,LLH message and publish them as ros message
     * Example:
     * $FP,LLH,1,2197,126191.765,47.398826818,8.458494107,457.518,0.31537,1.0076,0.072696,-0.080012,0.0067274,-0.011602*4E\r\n
     *
     * @param[in] tokens message split in tokens
     */
    void ConvertTokensAndPublish(const std::vector<std::string>& tokens) final;

   private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub_;
    const std::string header_ = "LLH";
    static constexpr const int kVersion_ = 1;
};
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_CONVERTER_LLH__
