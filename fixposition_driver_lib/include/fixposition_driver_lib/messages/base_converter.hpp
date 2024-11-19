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

template <typename MessageType> class NmeaConverter : public BaseAsciiConverter {
    public:
        using Observer = std::function<void(const MessageType&)>;

        /**
         * @brief Construct a new Fixposition Msg Converter object
         *
         */
        
        NmeaConverter() : BaseAsciiConverter() {}

        ~NmeaConverter() = default;

        /**
         * @brief Comma Delimited FP_A message, convert to Data structs and, if available, call observers
         *
         * @param[in] state state message as string
         * @return FP_A message struct
         */
        void ConvertTokens(const std::vector<std::string>& tokens) {
            msg_.ConvertFromTokens(tokens);

            // process all observers
            for (auto& ob : obs_) {
                ob(msg_);
            }
        };

        /**
         * @brief Add Observer to call at the end of ConvertTokens()
         *
         * @param[in] ob
         */
        void AddObserver(Observer ob) { obs_.push_back(ob); }

    private:
        MessageType msg_;
        std::vector<Observer> obs_;
};

//===================================================

/**
 * @brief Parse status flag field
 *
 * @param[in] tokens list of tokens
 * @param[in] idx status flag index
 * @return int
 */
inline int ParseStatusFlag(const std::vector<std::string>& tokens, const int idx) {
    if (tokens.at(idx).empty()) {
        return -1;
    } else {
        return std::stoi(tokens.at(idx));
    }
}

/**
 * @brief Helper function to convert string into int. If string is empty then 0 is returned
 *
 * @param[in] in_str
 * @return int
 */
inline double StringToInt(const std::string& in_str) { return in_str.empty() ? 0 : std::stoi(in_str); }

/**
 * @brief Helper function to convert string into double. If string is empty then 0.0 is returned
 *
 * @param[in] in_str
 * @return double
 */
inline double StringToDouble(const std::string& in_str) { return in_str.empty() ? 0. : std::stod(in_str); }

/**
 * @brief Helper function to convert string into char. If string is empty then '0' is returned
 *
 * @param[in] in_str
 * @return int
 */
inline double StringToChar(const std::string& in_str) { return in_str.empty() ? 0 : in_str[0]; }

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

/**
 * @brief Build a 3x3 covariance matrix
 *
 * [xx, xy, xz,
 *  xy, yy, yz,
 *  xz, yz, zz]
 *
 * @param[in] xx
 * @param[in] yy
 * @param[in] zz
 * @param[in] xy
 * @param[in] yz
 * @param[in] xz
 * @return Eigen::Matrix<double, 3, 3> the 3x3 matrix
 */
inline Eigen::Matrix<double, 3, 3> BuildCovMat3D(const double xx, const double yy, const double zz, 
                                                 const double xy, const double yz, const double xz) {
    Eigen::Matrix<double, 3, 3> cov;
    cov.setZero();

    // Diagonals
    cov(0, 0) = xx;   // 0
    cov(1, 1) = yy;   // 4
    cov(2, 2) = zz;   // 8

    // Rest of values
    cov(1, 0) = cov(0, 1) = xy; // 1 = 3
    cov(2, 1) = cov(1, 2) = yz; // 2 = 6
    cov(2, 0) = cov(0, 2) = xz; // 5 = 7

    return cov;
}

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
    cov.setZero();

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

/**
 * @brief Convert radians to degrees
 *
 * @tparam T data type
 * @param radians angle in [rad]
 *
 * @returns the angle in [deg]
 */
template <typename T>
constexpr inline T RadToDeg(T radians) {
    static_assert(std::is_floating_point<T>::value, "Only floating point types allowed!");
    return radians * 57.295779513082320876798154814105;
}

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_BASE_CONVERTER__
