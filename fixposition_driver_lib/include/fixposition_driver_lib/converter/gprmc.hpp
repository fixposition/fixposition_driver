/**
 *  @file
 *  @brief Declaration of GprmcConverter
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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_GPRMC__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_GPRMC__

/* SYSTEM / STL */

/* EXTERNAL */

/* PACKAGE */
#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

class GprmcConverter : public BaseAsciiConverter {
   public:
    using GprmcObserver = std::function<void(const GprmcData&)>;
    /**
     * @brief Construct a new GprmcConverter
     *
     */
    GprmcConverter() {}

    ~GprmcConverter() = default;

    /**
     * @brief Take comma-delimited tokens of GPRMC message, convert to Data structs and if available,
     * call observers
     * Example:
     * $GPRMC,151227.40,A,4723.54036,N,00826.88672,E,0.0,81.6,111022,,,R*7C\r\n
     *
     * @param[in] tokens message split in tokens
     */
    void ConvertTokens(const std::vector<std::string>& tokens) final;

    /**
     * @brief Add Observer to call at the end of ConvertTokens()
     *
     * @param[in] ob
     */
    void AddObserver(GprmcObserver ob) { obs_.push_back(ob); }

   private:
    GprmcData msg_;
    std::vector<GprmcObserver> obs_;
    const std::string header_ = "LLH";
    static constexpr const int kSize_ = 13;
};
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_GPRMC__
