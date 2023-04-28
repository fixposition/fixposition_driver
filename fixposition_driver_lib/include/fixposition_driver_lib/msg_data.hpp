/**
 *  @file
 *  @brief Declaration of Data types
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_LIB_MSG_DATA__
#define __FIXPOSITION_DRIVER_LIB_MSG_DATA__

/* EXTERNAL */
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/* PACKAGE */
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

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

struct VrtkData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    times::GpsTime stamp;
    std::string frame_id;
    std::string pose_frame;
    std::string kin_frame;
    PoseWithCovData pose;
    TwistWithCovData velocity;
    Eigen::Vector3d acceleration;
    int fusion_status;
    int imu_bias_status;
    int gnss1_status;
    int gnss2_status;
    int wheelspeed_status;
    std::string version;
    VrtkData()
        : fusion_status(-1),
          imu_bias_status(-1),
          gnss1_status(-1),
          gnss2_status(-1),
          wheelspeed_status(-1),
          version("") {
        acceleration.setZero();
    }
};

struct NavSatStatusData {
    enum class Status : int8_t {
        STATUS_NO_FIX = -1,   // # unable to fix position
        STATUS_FIX = 0,       // # unaugmented fix
        STATUS_SBAS_FIX = 1,  // # with satellite-based augmentation
        STATUS_GBAS_FIX = 2,  // # with ground-based augmentation
    };
    int8_t status;
    uint16_t service;
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

}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_LIB_MSG_DATA__
