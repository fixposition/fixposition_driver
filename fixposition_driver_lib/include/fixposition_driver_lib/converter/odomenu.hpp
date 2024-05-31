/**
 *  @file
 *  @brief Declaration of OdomenuConverter
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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_ODOMENU__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_ODOMENU__

/* SYSTEM / STL */

/* EXTERNAL */
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/* PACKAGE */

#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

class OdomenuConverter : public BaseAsciiConverter {
   public:
    /**
     * @brief Data for Messages published from the ODOMENU msg
     *
     */
    struct Msgs {
        OdometryData odometry;
        Eigen::Vector3d eul;
    };

    using OdomenuObserver = std::function<void(const Msgs&)>;

    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
    OdomenuConverter() : BaseAsciiConverter() {}

    ~OdomenuConverter() = default;

    /**
     * @brief Comma Delimited FP,ODOMENU message, convert to Data structs and if available,
     * call observers
     * Example:
     * $FP,ODOMENU,1,2180,298591.500000,-1.8339,2.6517,-0.0584,0.556794,-0.042551,-0.007850,0.829523,2.2993,
     * -1.6994,-0.0222,0.20063,0.08621,-1.21972,-3.6947,-3.3827,9.7482,4,1,8,8,1,0.00415,0.00946,0.00746,
     * -0.00149,-0.00084,0.00025,0.00003,0.00003,0.00012,0.00000,0.00000,0.00000,0.01742,0.01146,0.01612,
     * -0.00550,-0.00007,-0.00050*74\r\n
     *
     * @param[in] state state message as string
     * @return nav_msgs::Odometry message
     */
    virtual void ConvertTokens(const std::vector<std::string>& tokens) final;

    /**
     * @brief Add Observer to call at the end of ConvertTokens()
     *
     * @param[in] ob
     */
    void AddObserver(OdomenuObserver ob) { obs_.push_back(ob); }

   private:
    const std::string header_ = "ODOMENU";
    static constexpr const int kVersion_ = 1;
    static constexpr const int kSize_ = 44;

    Msgs msgs_;
    std::vector<OdomenuObserver> obs_;
};

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_ODOMENU__
