/**
 *  @file
 *  @brief Base Converter to define the interfaces
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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_BASE_CONVERTER__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_BASE_CONVERTER__

/* SYSTEM / STL */
#include <iostream>
#include <vector>

/* EXTERNAL */
#include <eigen3/Eigen/Geometry>

/* PACKAGE */
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {
class BaseAsciiConverter {
   public:
    BaseAsciiConverter() = default;
    ~BaseAsciiConverter() = default;

    /**
     * @brief Virtual interface to convert the split tokens into ros messages
     *
     * @param[in] tokens vector of strings split by comma
     */
    virtual void ConvertTokens(const std::vector<std::string>& tokens) = 0;

};

//===================================================

/**
 * @brief Helper function to convert string into double. If string is empty then 0.0 is returned
 *
 * @param[in] in_str
 * @return double
 */
inline double StringToDouble(const std::string& in_str) { return in_str.empty() ? 0. : std::stod(in_str); }

/**
 * @brief Make sure the quaternion is unit quaternion
 *
 * @param[in] quat Eigen::Quaterniond to be checked
 * @return true valid quaternion
 * @return false invalid quaternion
 */
inline bool CheckQuat(const Eigen::Quaterniond& quat) { return abs(quat.squaredNorm() - 1.0) <= 1e-3; }

/**
 * @brief Convert 3 string values into a Eigen::Vector3d
 *
 * @param[in] x
 * @param[in] y
 * @param[in] z
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d Vector3ToEigen(const std::string& x, const std::string& y, const std::string& z) {
    return Eigen::Vector3d(StringToDouble(x), StringToDouble(y), StringToDouble(z));
}

/**
 * @brief convert 4 string values into a Eigen::Quaterniond
 *
 * @param[in] w
 * @param[in] x
 * @param[in] y
 * @param[in] z
 * @return Eigen::Quaterniond
 */
inline Eigen::Quaterniond Vector4ToEigen(const std::string& w, const std::string& x, const std::string& y,
                                         const std::string& z) {
    return Eigen::Quaterniond(StringToDouble(w), StringToDouble(x), StringToDouble(y), StringToDouble(z));
}

/**
 * @brief Helper function to convert strings containing gps_wno and gps_tow into ros::Time
 *
 * @param[in] gps_wno
 * @param[in] gps_tow
 * @return ros::Time
 */
inline times::GpsTime ConvertGpsTime(const std::string& gps_wno, const std::string& gps_tow) {
    if (!gps_wno.empty() && !gps_tow.empty()) {
        return times::GpsTime(std::stoi(gps_wno), std::stod(gps_tow));
    } else {
        return times::GpsTime(0, 0);
    }
}
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_BASE_CONVERTER__
