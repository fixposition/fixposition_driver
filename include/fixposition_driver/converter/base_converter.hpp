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
    void ConvertStringAndPublish(const std::string& in_string);

    /**
     * @brief Virtual interface to convert the split tokens into ros messages
     *
     * @param[in] tokens vector of strings split by comma
     */
    virtual void ConvertTokensAndPublish(const std::vector<std::string>& tokens) = 0;

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
inline double StringToDouble(const std::string& in_str) { return in_str.empty() ? 0. : std::stod(in_str); }

/**
 * @brief Make sure the quaternion is unit quaternion
 *
 * @param[in] quat geometry_msgs::Quaternion to be checked
 * @return true valid quaternion
 * @return false invalid quaternion
 */
inline bool CheckQuat(const geometry_msgs::Quaternion& quat) {
    return abs(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z - 1.0) <= 1e-3;
}

/**
 * @brief convert 3 string values into a geometry_msgs::Vector3
 *
 * @param[out] msg
 * @param[in] x
 * @param[in] y
 * @param[in] z
 */
inline void Vector3ToMsg(geometry_msgs::Vector3& msg, const std::string& x, const std::string& y,
                         const std::string& z) {
    msg.x = StringToDouble(x);
    msg.y = StringToDouble(y);
    msg.z = StringToDouble(z);
}

/**
 * @brief convert 3 string values into a geometry_msgs::Point
 *
 * @param[out] msg
 * @param[in] x
 * @param[in] y
 * @param[in] z
 */
inline void Vector3ToMsg(geometry_msgs::Point& msg, const std::string& x, const std::string& y, const std::string& z) {
    msg.x = StringToDouble(x);
    msg.y = StringToDouble(y);
    msg.z = StringToDouble(z);
}

/**
 * @brief convert 3 string values into a geometry_msgs::Quaternion
 *
 * @param[out] msg
 * @param[in] w
 * @param[in] x
 * @param[in] y
 * @param[in] z
 */
inline void Vector4ToMsg(geometry_msgs::Quaternion& msg, const std::string& w, const std::string& x,
                         const std::string& y, const std::string& z) {
    msg.w = StringToDouble(w);
    msg.x = StringToDouble(x);
    msg.y = StringToDouble(y);
    msg.z = StringToDouble(z);
}

/**
 * @brief convert 3 string values into a geometry_msgs::Quaternion
 *
 * @param[out] msg
 * @param[in] quat
 */
inline void QuatToMsg(geometry_msgs::Quaternion& msg, const Eigen::Quaterniond& quat) {
    msg.w = quat.w();
    msg.x = quat.x();
    msg.y = quat.y();
    msg.z = quat.z();
}

/**
 * @brief geometry_msgs::Point to geometry_msgs::Vector3
 *
 * @param[out] msg
 * @param[in] in_msg
 */
inline void PointToVector3(geometry_msgs::Vector3& msg, const geometry_msgs::Point& in_msg) {
    msg.x = in_msg.x;
    msg.y = in_msg.y;
    msg.z = in_msg.z;
}

/**
 * @brief Helper function to convert strings containing gps_wno and gps_tow into ros::Time
 *
 * @param[in] gps_wno
 * @param[in] gps_tow
 * @return ros::Time
 */
inline ros::Time ConvertGpsTime(const std::string& gps_wno, const std::string& gps_tow) {
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
