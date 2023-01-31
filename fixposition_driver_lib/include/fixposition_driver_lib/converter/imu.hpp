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
#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_IMU__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_IMU__

/* EXTERNAL */
#include <eigen3/Eigen/Geometry>

/* PACKAGE */
#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/converter/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

class ImuConverter : public BaseConverter {
   public:
    using ImuObserver = std::function<void(const ImuData&)>;
    /**
     * @brief Construct a new ImuConverter
     *
     */
    ImuConverter(const bool bias_correction)
        : header_(bias_correction ? "CORRIMU" : "RAWIMU"), bias_correction_(bias_correction) {}

    ~ImuConverter() = default;

    bool CheckHeaderAndVersion(const std::string msg_header, const std::string msg_version) final {
        return msg_header == header_ && std::stoi(msg_version) == kVersion_;
    }

    /**
     * @brief Take comma-delimited tokens of FP,RAWIMU or FP,RAWIMU message, convert to Data structs and if available,
     * call observers
     * Example:
     * $FP,RAWIMU,1,2197,126191.777855,-0.199914,0.472851,9.917973,0.023436,0.007723,0.002131*34
     * $FP,CORRIMU,1,2197,126191.777855,-0.195224,0.393969,9.869998,0.013342,-0.004620,-0.000728*7D
     *
     * @param[in] tokens message split in tokens
     */
    void ConvertTokens(const std::vector<std::string>& tokens) final;

    /**
     * @brief Add Observer to call at the end of ConvertTokens()
     *
     * @param[in] ob
     */
    void AddObserver(ImuObserver ob) { obs_.push_back(ob); }

   private:
    ImuData msg_;
    std::vector<ImuObserver> obs_;
    const std::string header_;

    const bool bias_correction_;
    static constexpr const int kVersion_ = 1;
};
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_IMU__
