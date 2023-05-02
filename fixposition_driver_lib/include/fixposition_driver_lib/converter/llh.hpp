/**
 *  @file
 *  @brief Declaration of LlhConverter
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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_LLH__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_LLH__

/* SYSTEM / STL */

/* EXTERNAL */

/* PACKAGE */
#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

class LlhConverter : public BaseAsciiConverter {
   public:
    using LlhObserver = std::function<void(const NavSatFixData&)>;
    /**
     * @brief Construct a new LlhConverter
     *
     */
    LlhConverter() {}

    ~LlhConverter() = default;

    /**
     * @brief Take comma-delimited tokens of FP,LLH message, convert to Data structs and if available,
     * call observers
     * Example:
     * $FP,LLH,1,2197,126191.765,47.398826818,8.458494107,457.518,0.31537,1.0076,0.072696,-0.080012,0.0067274,-0.011602*4E\r\n
     *
     * @param[in] tokens message split in tokens
     */
    void ConvertTokens(const std::vector<std::string>& tokens) final;

    /**
     * @brief Add Observer to call at the end of ConvertTokens()
     *
     * @param[in] ob
     */
    void AddObserver(LlhObserver ob) { obs_.push_back(ob); }

   private:
    NavSatFixData msg_;
    std::vector<LlhObserver> obs_;
    const std::string header_ = "LLH";
    static constexpr const int kVersion_ = 1;
    static constexpr const int kSize_ = 14;
};
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_LLH__
