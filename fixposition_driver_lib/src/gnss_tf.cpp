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

/* PACKAGE */
#include <fixposition_driver_lib/gnss_tf.hpp>

namespace fixposition {

namespace Constants {
// static constexpr values representing the earth WGS84
// https://en.wikipedia.org/wiki/World_Geodetic_System#Defining_Parameters
static constexpr double wgs84_a = 6378137.0;           //!< earth radius major axis [m]
static constexpr double wgs84_b = 6356752.314245;      //!< earth radius minor axis [m]
static constexpr double wgs84_inv_f = 298.257223563;   //!< 1/f inverse of flattening parameter
static constexpr double wgs84_e_2 = 6.69437999014e-3;  //!< first eccentricity Squared

static constexpr double wgs84_a_2 = wgs84_a * wgs84_a;                //!< a_^2
static constexpr double wgs84_b_2 = wgs84_b * wgs84_b;                //!< a_^2
static constexpr double wgs84_e_prime_2 = wgs84_a_2 / wgs84_b_2 - 1;  //!< e'^2 second eccentricity squared
}  // namespace Constants

/**
 * @details
 *
 * \f[
 *
 * \text{Rotate a vector from ECEF to ENU:} \\
 * V_{ENU} = M \cdot V_{ECEF} \\
 * \text{Rotate a covariance matrix from ECEF to ENU:} \\
 * Cov_{ENU} = M \cdot Cov_{ECEF} \cdot M^{T} \\
 *
 * \f]
 *
 * Reference https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
 *
 */
Eigen::Matrix3d RotEnuEcef(double lat, double lon) {
    const double s_lon = sin(lon);
    const double c_lon = cos(lon);
    const double s_lat = sin(lat);
    const double c_lat = cos(lat);
    const double m00 = -s_lon;
    const double m01 = c_lon;
    const double m02 = 0;
    const double m10 = -c_lon * s_lat;
    const double m11 = -s_lon * s_lat;
    const double m12 = c_lat;
    const double m20 = c_lon * c_lat;
    const double m21 = s_lon * c_lat;
    const double m22 = s_lat;

    Eigen::Matrix3d res;
    // This initializes the matrix row by row
    res << m00, m01, m02, m10, m11, m12, m20, m21, m22;

    return res;
}

/**
 * @details
 *
 * \f[
 *
 * \text{Rotate a vector from ECEF to ENU:} \\
 * V_{ENU} = M \cdot V_{ECEF} \\
 * \text{Rotate a covariance matrix from ECEF to ENU:} \\
 * Cov_{ENU} = M \cdot Cov_{ECEF} \cdot M^{T} \\
 *
 * \f]
 *
 */
Eigen::Matrix3d RotEnuEcef(const Eigen::Vector3d &ecef) {
    const Eigen::Vector3d wgs84llh = TfWgs84LlhEcef(ecef);
    return RotEnuEcef(wgs84llh.x(), wgs84llh.y());
}

Eigen::Matrix3d RotNedEnu() {
    /**
     * | 0, 1, 0 |
     * | 1, 0, 0 |
     * | 0, 0,-1 |
     *
     */

    Eigen::Matrix3d rot_ned_enu;
    rot_ned_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    return rot_ned_enu;
}

Eigen::Matrix3d RotNedEcef(double lat, double lon) { return RotNedEnu() * RotEnuEcef(lat, lon); }

Eigen::Matrix3d RotNedEcef(const Eigen::Vector3d &ecef) {
    const Eigen::Vector3d wgs84llh = TfWgs84LlhEcef(ecef);
    return RotNedEcef(wgs84llh.x(), wgs84llh.y());
}

Eigen::Vector3d TfEnuEcef(const Eigen::Vector3d &ecef, const Eigen::Vector3d &wgs84llh_ref) {
    const Eigen::Vector3d ecef_ref = TfEcefWgs84Llh(wgs84llh_ref);
    return RotEnuEcef(wgs84llh_ref.x(), wgs84llh_ref.y()) * (ecef - ecef_ref);
}

Eigen::Vector3d TfEcefEnu(const Eigen::Vector3d &enu, const Eigen::Vector3d &wgs84llh_ref) {
    const Eigen::Vector3d ecef_ref = TfEcefWgs84Llh(wgs84llh_ref);
    return ecef_ref + RotEnuEcef(wgs84llh_ref.x(), wgs84llh_ref.y()).transpose() * enu;
}

Eigen::Vector3d TfNedEcef(const Eigen::Vector3d &ecef, const Eigen::Vector3d &wgs84llh_ref) {
    const Eigen::Vector3d ecef_ref = TfEcefWgs84Llh(wgs84llh_ref);
    return RotNedEcef(wgs84llh_ref.x(), wgs84llh_ref.y()) * (ecef - ecef_ref);
}

Eigen::Vector3d TfEcefNed(const Eigen::Vector3d &ned, const Eigen::Vector3d &wgs84llh_ref) {
    const Eigen::Vector3d ecef_ref = TfEcefWgs84Llh(wgs84llh_ref);
    return ecef_ref + RotNedEcef(wgs84llh_ref.x(), wgs84llh_ref.y()).transpose() * ned;
}
/**
 * @details Implementation based on paper
 * https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#The_application_of_Ferrari's_solution
 *
 */
Eigen::Vector3d TfWgs84LlhEcef(const Eigen::Vector3d &ecef) {
    const double x = ecef.x();
    const double y = ecef.y();
    const double z = ecef.z();

    const double x_2 = x * x;  //!< x^2
    const double y_2 = y * y;  //!< y^2
    const double z_2 = z * z;  //!< z^2

    const double r_2 = (x_2 + y_2);  //!< r^2
    const double r = sqrt(r_2);      //!< r

    const double F = 54.0 * Constants::wgs84_b_2 * z_2;
    const double G =
        r_2 + (1 - Constants::wgs84_e_2) * z_2 - Constants::wgs84_e_2 * (Constants::wgs84_a_2 - Constants::wgs84_b_2);
    const double c = Constants::wgs84_e_2 * Constants::wgs84_e_2 * F * r_2 / (G * G * G);
    const double s = cbrt(1 + c + sqrt(c * c + 2 * c));
    const double P = F / (3.0 * (s + 1.0 + 1.0 / s) * (s + 1.0 + 1.0 / s) * G * G);
    const double Q = sqrt(1 + 2 * Constants::wgs84_e_2 * Constants::wgs84_e_2 * P);
    const double r0 = -P * Constants::wgs84_e_2 * r / (1 + Q) +
                      sqrt(0.5 * Constants::wgs84_a_2 * (1.0 + 1.0 / Q) -
                           (P * (1 - Constants::wgs84_e_2) * z_2 / (Q + Q * Q)) - 0.5 * P * r_2);
    const double t1 = (r - Constants::wgs84_e_2 * r0);
    const double t1_2 = t1 * t1;
    const double U = sqrt(t1_2 + z_2);
    const double V = sqrt(t1_2 + (1 - Constants::wgs84_e_2) * z_2);
    const double a_V = Constants::wgs84_a * V;
    const double z0 = Constants::wgs84_b_2 * z / a_V;

    const double h = U * (1 - Constants::wgs84_b_2 / a_V);
    const double lat = atan((z + Constants::wgs84_e_prime_2 * z0) / r);
    const double lon = atan2(y, x);

    return Eigen::Vector3d(lat, lon, h);
}

Eigen::Vector3d TfEcefWgs84Llh(const Eigen::Vector3d &wgs84llh) {
    const double lat = wgs84llh.x();
    const double lon = wgs84llh.y();
    const double height = wgs84llh.z();
    const double s_lat = sin(lat);
    const double c_lat = cos(lat);
    const double s_lon = sin(lon);
    const double c_lon = cos(lon);
    // N is in meters
    const double n = Constants::wgs84_a / sqrt(1.0 - Constants::wgs84_e_2 * s_lat * s_lat);
    const double n_plus_height = n + height;

    Eigen::Vector3d ecef;
    ecef.x() = n_plus_height * c_lat * c_lon;
    ecef.y() = n_plus_height * c_lat * s_lon;
    ecef.z() = (n * (1 - Constants::wgs84_e_2) + height) * s_lat;

    return ecef;
}

Eigen::Vector3d EcefPoseToEnuEul(const Eigen::Vector3d &ecef_p, const Eigen::Matrix3d &ecef_r) {
    //! Rotation Matrix to convert to ENU frame
    const Eigen::Matrix3d rot_enu_ecef = RotEnuEcef(ecef_p);
    //! Convert the Pose into ENU frame
    const Eigen::Matrix3d rot_enu_body = rot_enu_ecef * ecef_r;
    //! Convert Rotation Matrix into Yaw-Pitch-Roll
    return RotToEul(rot_enu_body);
}

}  // namespace fixposition
