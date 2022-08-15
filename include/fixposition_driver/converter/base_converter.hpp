/**
 *  @file
 *  @brief Base Converter to define the interfaces
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_CONVERTER_BASE_CONVERTER__
#define __FIXPOSITION_DRIVER_CONVERTER_BASE_CONVERTER__

/* SYSTEM / STL */
#include <vector>

/* EXTERNAL */
#include <eigen3/Eigen/Geometry>

/* ROS */
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/console.h>

/* PACKAGE */
#include <fixposition_driver/time_conversions.hpp>

namespace fixposition {
class BaseConverter {
   public:
    BaseConverter() = default;
    ~BaseConverter() = default;

    /**
     * @brief API Interface of all converters, to convert the Input string into Ros messages and publish them
     *
     * @param[in] in_string string of comma delimited message to convert
     */
    void ConvertStringAndPublish(const std::string &in_string);

    /**
     * @brief Virtual interface to convert the split tokens into ros messages
     *
     * @param[in] tokens vector of strings split by comma
     */
    virtual void ConvertTokensAndPublish(const std::vector<std::string> &tokens) = 0;

    /**
     * @brief Check the header and version of the corresponding message to make sure it is correct
     *
     * @param[in] msg_header expected header
     * @param[in] msg_version expected version
     * @return true correct
     * @return false incorrect
     */
    virtual bool CheckHeaderAndVersion(const std::string msg_header, const std::string msg_version) = 0;
};

/**
 * @brief Helper function to convert string into double. If string is empty then 0.0 is returned
 *
 * @param[in] in_str
 * @return double
 */
inline double StringToDouble(const std::string &in_str) { return in_str.empty() ? 0. : std::stod(in_str); }

/**
 * @brief Make sure the quaternion is unit quaternion
 *
 * @param[in] quat geometry_msgs::Quaternion to be checked
 * @return true valid quaternion
 * @return false invalid quaternion
 */
inline bool CheckQuat(const geometry_msgs::Quaternion &quat) {
    return abs(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z - 1.0) <= 1e-3;
}

/**
 * @brief convert 3 string values into a geometry_msgs::Vector3
 *
 * @param[in] x
 * @param[in] y
 * @param[in] z
 * @return geometry_msgs::Vector3
 */
inline geometry_msgs::Vector3 Vector3ToMsg(const std::string &x, const std::string &y, const std::string &z) {
    geometry_msgs::Vector3 msg;
    msg.x = StringToDouble(x);
    msg.y = StringToDouble(y);
    msg.z = StringToDouble(z);
    return msg;
}

/**
 * @brief convert 3 string values into a geometry_msgs::Point
 *
 * @param[in] x
 * @param[in] y
 * @param[in] z
 * @return geometry_msgs::Point
 */
inline geometry_msgs::Point Vector3ToPointMsg(const std::string &x, const std::string &y, const std::string &z) {
    geometry_msgs::Point msg;
    msg.x = StringToDouble(x);
    msg.y = StringToDouble(y);
    msg.z = StringToDouble(z);
    return msg;
}

/**
 * @brief convert 4 string values into a geometry_msgs::Quaternion
 *
 * @param[in] w
 * @param[in] x
 * @param[in] y
 * @param[in] z
 * @return geometry_msgs::Quaternion
 */
inline geometry_msgs::Quaternion Vector4ToMsg(const std::string &w, const std::string &x, const std::string &y,
                                              const std::string &z) {
    geometry_msgs::Quaternion msg;
    msg.w = StringToDouble(w);
    msg.x = StringToDouble(x);
    msg.y = StringToDouble(y);
    msg.z = StringToDouble(z);
    return msg;
}

/**
 * @brief convert Eigen::Quaterniond into a geometry_msgs::Quaternion
 *
 * @param[in] quat
 * @return geometry_msgs::Quaternion
 */
inline geometry_msgs::Quaternion EigenToQuatMsg(const Eigen::Quaterniond &quat) {
    geometry_msgs::Quaternion msg;
    msg.w = quat.w();
    msg.x = quat.x();
    msg.y = quat.y();
    msg.z = quat.z();
    return msg;
}

/**
 * @brief Convert 3 string values into a Eigen::Vector3d
 *
 * @param[in] x
 * @param[in] y
 * @param[in] z
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d Vector3ToEigen(const std::string &x, const std::string &y, const std::string &z) {
    return Eigen::Vector3d(StringToDouble(x), StringToDouble(y), StringToDouble(z));
}

/**
 * @brief Convert Eigen::Vector3d into geometry_msgs::Point
 *
 * @param[in] v
 * @return geometry_msgs::Point
 */
inline geometry_msgs::Point EigenToPointMsg(const Eigen::Vector3d &v) {
    geometry_msgs::Point msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

/**
 * @brief Convert Eigen::Vector3d into geometry_msgs::Vector3
 *
 * @param[in] v
 * @return geometry_msgs::Vector3
 */
inline geometry_msgs::Vector3 EigenToVector3Msg(const Eigen::Vector3d &v) {
    geometry_msgs::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
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
inline Eigen::Quaterniond Vector4ToEigen(const std::string &w, const std::string &x, const std::string &y,
                                         const std::string &z) {
    return Eigen::Quaterniond(StringToDouble(w), StringToDouble(x), StringToDouble(y), StringToDouble(z));
}

/**
 * @brief Helper function to convert strings containing gps_wno and gps_tow into ros::Time
 *
 * @param[in] gps_wno
 * @param[in] gps_tow
 * @return ros::Time
 */
inline ros::Time ConvertGpsTime(const std::string &gps_wno, const std::string &gps_tow) {
    if (!gps_wno.empty() && gps_tow.empty()) {
        const times::GpsTime gps_time(std::stoi(gps_wno), std::stod(gps_tow));
        return times::GpsTimeToRosTime(gps_time);
    } else {
        ROS_DEBUG_STREAM("GPS time empty. Replacing with current ROS time.");
        return ros::Time::now();
    }
}
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_CONVERTER_BASE_CONVERTER__
