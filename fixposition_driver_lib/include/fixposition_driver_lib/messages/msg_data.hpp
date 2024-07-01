/**
 *  @file
 *  @brief Declaration of Data types
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

#ifndef __FIXPOSITION_DRIVER_LIB_MSG_DATA__
#define __FIXPOSITION_DRIVER_LIB_MSG_DATA__

/* PACKAGE */
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

enum class FusionStatus : int {
    NOT_STARTED = 0,
    VISION_ONLY = 1,
    VISUAL_INERTIAL_FUSION = 2,
    INERTIAL_GNSS_FUSION = 3,
    VISUAL_INERTIAL_GNSS_FUSION = 4,
};

enum class ImuStatus : int {
    NOT_CONVERGED = 0,
    BIAS_CONVERGED = 1,
};

enum class GnssStatus : int {
    FIX_TYPE_UNKNOWN = 0,
    FIX_TYPE_NOFIX = 1,
    FIX_TYPE_DRONLY = 2,
    FIX_TYPE_TIME = 3,
    FIX_TYPE_S2D = 4,
    FIX_TYPE_S3D = 5,
    FIX_TYPE_S3D_DR = 6,
    FIX_TYPE_RTK_FLOAT = 7,
    FIX_TYPE_RTK_FIXED = 8,
    FIX_TYPE_RTK_FLOAT_DR = 9,
    FIX_TYPE_RTK_FIXED_DR = 10,
};

enum class WheelspeedStatus : int {
    NOT_ENABLED = -1,
    NOT_CONVERGED = 0,
    WS_CONVERGED = 1,
};

struct ImuData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    times::GpsTime stamp;
    std::string frame_id;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
    ImuData() {
        linear_acceleration.setZero();
        angular_velocity.setZero();
    }
};

struct TfData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    times::GpsTime stamp;
    std::string frame_id;
    std::string child_frame_id;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    TfData() : frame_id(""), child_frame_id("") {
        translation.setZero();
        rotation.setIdentity();
    }
};

struct PoseWithCovData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Matrix<double, 6, 6> cov;
    PoseWithCovData() {
        position.setZero();
        orientation.setIdentity();
        cov.setZero();
    }
};

struct TwistWithCovData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
    Eigen::Matrix<double, 6, 6> cov;
    TwistWithCovData() {
        linear.setZero();
        angular.setZero();
        cov.setZero();
    }
};

struct OdometryData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    times::GpsTime stamp;
    std::string frame_id;
    std::string child_frame_id;
    PoseWithCovData pose;
    TwistWithCovData twist;
};

struct NavSatStatusData {
    enum class Status : int8_t {
        STATUS_NO_FIX   = -1, // Unable to fix position
        STATUS_FIX      =  0, // Unaugmented fix
        STATUS_SBAS_FIX =  1, // With satellite-based augmentation
        STATUS_GBAS_FIX =  2, // With ground-based augmentation
    };

    enum class Service : uint16_t {
        SERVICE_NONE    =  0,
        SERVICE_GPS     =  1,
        SERVICE_GLONASS =  2,
        SERVICE_COMPASS =  4, // includes BeiDou.
        SERVICE_GALILEO =  8,
        SERVICE_ALL     = 15,
    };
    
    int8_t status;
    uint16_t service;
    NavSatStatusData() : status(static_cast<int8_t>(Status::STATUS_NO_FIX)), service(0) {}
};

struct NavSatFixData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    times::GpsTime stamp;
    std::string frame_id;
    NavSatStatusData status;
    double latitude;
    double longitude;
    double altitude;
    Eigen::Matrix<double, 3, 3> cov;
    int position_covariance_type;
    NavSatFixData() : latitude(0.0), longitude(0.0), altitude(0.0), position_covariance_type(0) { cov.setZero(); }
};

struct GpggaData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string time;
    double latitude;
    double longitude;
    double altitude;
    Eigen::Matrix<double, 3, 3> cov;
    int position_covariance_type;
    bool valid;
    GpggaData() : latitude(0.0), longitude(0.0), altitude(0.0), position_covariance_type(0), valid(false) { cov.setZero(); }
};

struct GpzdaData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string time;
    std::string date;
    times::GpsTime stamp;
    bool valid;
    GpzdaData() : time(""), date(""), valid(false) {}
};

struct GprmcData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string time;
    std::string mode;
    double latitude;
    double longitude;
    double speed;
    double course;
    bool valid;
    GprmcData() : latitude(0.0), longitude(0.0), speed(0.0), course(0.0), valid(false) {}
};

}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_LIB_MSG_DATA__
