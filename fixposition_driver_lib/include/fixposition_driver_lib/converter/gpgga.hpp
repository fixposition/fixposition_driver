/**
 *  @file
 *  @brief Declaration of GpggaConverter
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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_GPGGA__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_GPGGA__

/* SYSTEM / STL */

/* EXTERNAL */

/* PACKAGE */
#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

class GpggaConverter : public BaseAsciiConverter {
   public:
    using GpggaObserver = std::function<void(const NavSatFixData&)>;
    /**
     * @brief Construct a new GpggaConverter
     *
     */
    GpggaConverter() {}

    ~GpggaConverter() = default;

    /**
     * @brief Take comma-delimited tokens of GPGGA message, convert to Data structs and if available,
     * call observers
     * Example:
     * $GPGGA,151229.40,4723.54108,N,00826.88485,E,4,12,00.98,473.5,M,,,,*3A\r\n
     *
     * @param[in] tokens message split in tokens
     */
    void ConvertTokens(const std::vector<std::string>& tokens) final;

    /**
     * @brief Add Observer to call at the end of ConvertTokens()
     *
     * @param[in] ob
     */
    void AddObserver(GpggaObserver ob) { obs_.push_back(ob); }

   private:
    NavSatFixData msg_;
    std::vector<GpggaObserver> obs_;
    const std::string header_ = "LLH";
    static constexpr const int kSize_ = 15;
};
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_GPGGA__
