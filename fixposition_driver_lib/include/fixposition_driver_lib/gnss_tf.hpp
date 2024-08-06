/**
 *  @file
 *  @brief Helper functions
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

#ifndef __FIXPOSITION_DRIVER_LIB_GNSS_TF__
#define __FIXPOSITION_DRIVER_LIB_GNSS_TF__

/* PACKAGE */
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

/**
 * @brief Calculate the rotation matrix from ECEF to
 * ENU for a given reference latitude/longitude.
 *
 * @param[in] lat Reference latitude [rad]
 * @param[in] lon Reference longitude [rad]
 * @return Eigen::Matrix3d Rotation matrix from ECEF -> ENU
 */
Eigen::Matrix3d RotEnuEcef(double lat, double lon);

/**
 * @brief Calculate the rotation matrix from ECEF to
 * ENU for a given reference origin.
 *
 * @param[in] in_pos Reference position in ECEF [m]
 * @return Eigen::Matrix3d Rotation matrix ECEF -> ENU
 */
Eigen::Matrix3d RotEnuEcef(const Eigen::Vector3d &ecef);

/**
 * @brief Returns rotation matrix between NED and ENU
 * @details | 0, 1, 0 |
 *          | 1, 0, 0 |
 *          | 0, 0,-1 |
 *
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d RotNedEnu();

/**
 * @brief Calculate the rotation matrix from ECEF to
 * NED for a given reference latitude/longitude.
 *
 * @param[in] lat Reference latitude [rad]
 * @param[in] lon Reference longitude [rad]
 * @return Eigen::Matrix3d Rotation matrix from ECEF -> NED
 */
Eigen::Matrix3d RotNedEcef(double lat, double lon);

/**
 * @brief Calculate the rotation matrix from ECEF to
 * NED for a given reference origin.
 *
 * @param[in] in_pos Reference position in ECEF [m]
 * @return Eigen::Matrix3d Rotation matrix ECEF -> NED
 */
Eigen::Matrix3d RotNedEcef(const Eigen::Vector3d &ecef);

/**
 * @brief Transform ECEF coordinate to ENU with specified ENU-origin
 *
 * @param[in] xyz ECEF position to be transsformed [m]
 * @param[in] wgs84llh_ref ENU-origin in Geodetic coordinates (Lat[rad], Lon[rad], Height[m])
 * @return Eigen::Vector3d Position in ENU coordinates
 */
Eigen::Vector3d TfEnuEcef(const Eigen::Vector3d &ecef, const Eigen::Vector3d &wgs84llh_ref);

/**
 * @brief  Transform ENU coordinate to ECEF with specified ENU-origin
 *
 * @param[in] enu ENU position to be transsformed [m]
 * @param[in] wgs84llh_ref ENU-origin in Geodetic coordinates (Lat[rad], Lon[rad], Height[m])
 * @return Eigen::Vector3d
 */
Eigen::Vector3d TfEcefEnu(const Eigen::Vector3d &enu, const Eigen::Vector3d &wgs84llh_ref);

Eigen::Vector3d TfNedEcef(const Eigen::Vector3d &ecef, const Eigen::Vector3d &wgs84llh_ref);
Eigen::Vector3d TfEcefNed(const Eigen::Vector3d &ned, const Eigen::Vector3d &wgs84llh_ref);

/**
 * @brief Convert Geodetic coordinates (latitude, longitude, height) to
 * ECEF (x, y, z).
 *
 * @param[in] wgs84llh Geodetic coordinates (Lat[rad], Lon[rad], Height[m])
 * @return Eigen::Vector3d ECEF coordinates [m]
 */
Eigen::Vector3d TfEcefWgs84Llh(const Eigen::Vector3d &wgs84llh);

/**
 * @brief Convert ECEF (x, y, z) coordinates to Lat Lon Height coordinates
 * (latitude, longitude, altitude).
 *
 * @param[in] ecef ECEF coordinates [m]
 * @return Eigen::Vector3d Geodetic coordinates (Lat[rad], Lon[rad], Height[m])
 */
Eigen::Vector3d TfWgs84LlhEcef(const Eigen::Vector3d &ecef);

/**
 * @brief Given Pose in ECEF, calculate Yaw-Pitch-Roll in ENU
 * @details Yaw will be -Pi/2 when X is pointing North, because ENU starts with East
 *
 * @param ecef_p 3D Position Vector in ECEF
 * @param ecef_r 3x3 Rotation matrix representing rotation Body --> ECEF
 * @return Eigen::Vector3d Yaw-Pitch-Roll in ENU
 * (Yaw will be -Pi/2 when X is pointing North, because ENU starts with

 */
Eigen::Vector3d EcefPoseToEnuEul(const Eigen::Vector3d &ecef_p, const Eigen::Matrix3d &ecef_r);

/**
 * @brief
 *
 * @param ecef_p 3D Position Vector in ECEF
 * @param ecef_r 3x3 Rotation matrix representing rotation Body --> ECEF
 * @return Eigen::Vector3d Yaw-Pitch-Roll in NED
 */
Eigen::Vector3d EcefPoseToNedEul(const Eigen::Vector3d &ecef_p, const Eigen::Matrix3d &ecef_r);

/**
 * @brief Vector4 quaternion to intrinsic Euler Angles in ZYX (yaw,pitch,roll)
 *
 * @param[in] quat Eigen::Quaterniond in w,i,j,k
 * @return Eigen::Vector3d intrinsic euler angle in ZYX (Yaw/Pitch/Roll) order
 */
inline Eigen::Vector3d QuatToEul(const Eigen::Quaterniond &q) {
    auto qw = q.w();
    auto qx = q.x();
    auto qy = q.y();
    auto qz = q.z();
    auto eul0 = atan2(2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
    auto eul1 = asin(-2.0 * (qx * qz - qw * qy));
    auto eul2 = atan2(2 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);

    Eigen::Vector3d eul;
    eul << eul0, eul1, eul2;

    return eul;
}

/**
 * @brief Rotation Matrix to intrinsic Euler Angles in ZYX (Yaw-Pitch-Roll) order in radian
 *
 * @param rot Eigen::Matrix3d
 * @return Eigen::Vector3d intrinsic euler angle in ZYX (Yaw-Pitch-Roll) order in radian
 */
inline Eigen::Vector3d RotToEul(const Eigen::Matrix3d &rot) {
    const Eigen::Quaterniond quat(rot);
    return QuatToEul(quat);
}

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_GNSS_TF__
