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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_TF__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_TF__

/* EXTERNAL */
#include <eigen3/Eigen/Geometry>

/* PACKAGE */
#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/converter/msg_data.hpp>

namespace fixposition {

class TfConverter : public BaseConverter {
   public:
    using TfObserver = std::function<void(const TfData&)>;
    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
    TfConverter() = default;

    ~TfConverter() = default;

    bool CheckHeaderAndVersion(const std::string msg_header, const std::string msg_version) final {
        return msg_header == header_ && std::stoi(msg_version) == kVersion_;
    }
    /**
     * @brief ake comma-delimited tokens of FP,TF message, convert to Data structs and if available,
     * call observers
     *
     * @param[in] state state message as string
     * @return nav_msgs::Odometry message
     */
    void ConvertTokens(const std::vector<std::string>& tokens) final;

    /**
     * @brief Add Observer to call at the end of ConvertTokens()
     *
     * @param[in] ob
     */
    void AddObserver(TfObserver ob) { obs_.push_back(ob); }

   private:
    TfData msg_;
    std::vector<TfObserver> obs_;

    const std::string header_ = "TF";

    static constexpr const int kVersion_ = 1;
};
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_TF__
