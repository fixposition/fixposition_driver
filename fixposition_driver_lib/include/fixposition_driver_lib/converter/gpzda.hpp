/**
 *  @file
 *  @brief Declaration of GpzdaConverter
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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_GPZDA__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_GPZDA__

/* SYSTEM / STL */

/* EXTERNAL */

/* PACKAGE */
#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>
#include <chrono>

namespace fixposition {

class GpzdaConverter : public BaseAsciiConverter {
   public:
    using GpzdaObserver = std::function<void(const GpzdaData&)>;
    /**
     * @brief Construct a new GpzdaConverter
     *
     */
    GpzdaConverter() {}

    ~GpzdaConverter() = default;

    /**
     * @brief Take comma-delimited tokens of GPZDA message, convert to Data structs and if available,
     * call observers
     * Example:
     * $GPZDA,090411.0001,10,10,2023,00,00*69\r\n
     *
     * @param[in] tokens message split in tokens
     */
    void ConvertTokens(const std::vector<std::string>& tokens) final;

    /**
     * @brief Add Observer to call at the end of ConvertTokens()
     *
     * @param[in] ob
     */
    void AddObserver(GpzdaObserver ob) { obs_.push_back(ob); }

   private:
    GpzdaData msg_;
    std::vector<GpzdaObserver> obs_;
    const std::string header_ = "LLH";
    static constexpr const int kSize_ = 7;
};
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_GPZDA__
