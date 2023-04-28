/**
 *  @file
 *  @brief Declaration of OdometryConverter
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
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
        OdometryData odometry_enu0;
        VrtkData vrtk;
        Eigen::Vector3d eul;
        TfData tf_ecef_poi;
        TfData tf_ecef_enu;
        TfData tf_ecef_enu0;
    };

    using OdometryObserver = std::function<void(const Msgs&)>;

    /**
     * @brief Construct a new Fixposition Msg Converter object
     *
     */
    OdometryConverter() : BaseAsciiConverter(), tf_ecef_enu0_set_(false) {
        msgs_.tf_ecef_enu0.frame_id = "ECEF";
        msgs_.tf_ecef_enu0.child_frame_id = "FP_ENU0";
    }

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

    //! transform between ECEF and ENU0
    bool tf_ecef_enu0_set_;  //!< flag to indicate if the tf is already set
    Eigen::Vector3d t_ecef_enu0_;
    Eigen::Quaterniond q_ecef_enu0_;

    Msgs msgs_;
    std::vector<OdometryObserver> obs_;
};

/**
 * @brief Build a 6x6 covariance matrix which is 2 independent 3x3 matrices
 *
 * [xx, xy, xz, 0, 0, 0,
 *  xy, yy, yz, 0, 0, 0,
 *  xz, yz, zz, 0, 0, 0,
 *  0, 0, 0, xx1, xy1, xz1,
 *  0, 0, 0, xy1, yy1, yz1,
 *  0, 0, 0, xz1, yz1, zz1]
 *
 * @param[in] xx
 * @param[in] yy
 * @param[in] zz
 * @param[in] xy
 * @param[in] yz
 * @param[in] xz
 * @param[in] xx1
 * @param[in] yy1
 * @param[in] zz1
 * @param[in] xy1
 * @param[in] yz1
 * @param[in] xz1
 * @return Eigen::Matrix<double, 6, 6> the 6x6 matrix
 */
inline Eigen::Matrix<double, 6, 6> BuildCovMat6D(const double xx, const double yy, const double zz, const double xy,
                                                 const double yz, const double xz, double xx1, const double yy1,
                                                 const double zz1, const double xy1, const double yz1, double xz1) {
    Eigen::Matrix<double, 6, 6> cov;
    // Diagonals
    cov(0, 0) = xx;   // 0
    cov(1, 1) = yy;   // 7
    cov(2, 2) = zz;   // 14
    cov(3, 3) = xx1;  // 21
    cov(4, 4) = yy1;  // 28
    cov(5, 5) = zz1;  // 35

    // Rest of values
    cov(1, 0) = cov(0, 1) = xy;   // 1 = 6
    cov(2, 1) = cov(1, 2) = yz;   // 8 = 13
    cov(2, 0) = cov(0, 2) = xz;   // 2 = 12
    cov(4, 3) = cov(3, 4) = xy1;  // 22 = 27
    cov(5, 4) = cov(4, 5) = yz1;  // 29 = 34
    cov(5, 3) = cov(3, 5) = xz1;  // 23 = 33

    return cov;
}

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_ODOMETRY__
