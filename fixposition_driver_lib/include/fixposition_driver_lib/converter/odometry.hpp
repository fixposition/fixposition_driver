/**
 *  @file
 *  @brief Declaration of OdometryConverter
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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_ODOMETRY__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_ODOMETRY__

/* SYSTEM / STL */

/* EXTERNAL */
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/* PACKAGE */

#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

class OdometryConverter : public BaseAsciiConverter {
   public:
    /**
     * @brief Data for Messages published from the ODOMETRY msg
     *
     */
    struct Msgs {
        OdometryData odometry;
        ImuData imu;
        VrtkData vrtk;
        TfData tf_ecef_poi;
        NavSatFixData odom_llh;
    };

    using OdometryObserver = std::function<void(const Msgs&)>;

    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
    OdometryConverter() : BaseAsciiConverter() {}

    ~OdometryConverter() = default;

    /**
     * @brief Comma Delimited FP,ODOMETRY message, convert to Data structs and if available,
     * call observers
     * Example:
     * $FP,ODOMETRY,1,2216,509791.426,4282251.9970,641470.7361,4668050.6007,-0.016059,0.359036,0.067523,
     * 0.930740,9.6271,0.0718,-0.0231,0.01118,0.00952,0.03600,1.1313,0.3863,9.5607,4,1,8,1,0.01411,0.00720,0.01547,
     * 0.00090,-0.00105,-0.00832,0.00021,0.00016,0.00019,-0.00000,0.00001,0.00008,0.07652,
     * 0.05768,0.05234,0.00309,-0.00001,0.00173,fp_release_vr2_2.46.1_124*7D
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
    void AddObserver(OdometryObserver ob) { obs_.push_back(ob); }

   private:
    const std::string header_ = "ODOMETRY";
    static constexpr const int kVersion_ = 2;
    static constexpr const int kSize_ = 45;

    Msgs msgs_;
    std::vector<OdometryObserver> obs_;
};

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_ODOMETRY__
