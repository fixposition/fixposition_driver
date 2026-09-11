/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Optional PROJ-backed ECEF to geodetic coordinate conversion
 */

#ifndef __FIXPOSITION_DRIVER_LIB_LLH_TRANSFORMER_HPP__
#define __FIXPOSITION_DRIVER_LIB_LLH_TRANSFORMER_HPP__

/* LIBC/STL */
#include <string>

/* EXTERNAL */
#include <fpsdk_common/ext/eigen_core.hpp>

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Convert ECEF coordinates to latitude, longitude and height
 *
 * The configured target must be either a three-dimensional geographic CRS or a compound CRS consisting of a
 * two-dimensional geographic CRS and a vertical CRS. Output is always latitude [rad], longitude [rad], height [m].
 * PROJ state is created per calling thread because a PROJ context must not be shared across threads.
 */
class LlhTransformer {
   public:
    /**
     * @brief Configure and validate the transformation
     *
     * @param[in]  enabled     Use PROJ when true; use the built-in WGS84 conversion when false
     * @param[in]  ecef_crs    Geocentric source CRS, normally "EPSG:4978"
     * @param[in]  llh_crs     Geographic 3D or geographic 2D plus vertical target CRS
     *
     * @returns true if disabled or if the PROJ transformation is available and valid
     */
    bool Init(bool enabled, const std::string& ecef_crs, const std::string& llh_crs);

    /**
     * @brief Convert ECEF [m] to latitude [rad], longitude [rad], height [m]
     *
     * @returns true on success, false if the configured PROJ operation cannot transform the coordinate
     */
    bool EcefToLlhRad(const Eigen::Vector3d& ecef, Eigen::Vector3d& llh_rad) const;

    bool enabled() const { return proj_enabled_; }
    const std::string& error() const { return error_; }

   private:
    bool proj_enabled_ = false;
    std::string ecef_crs_;
    std::string llh_crs_;
    std::string error_;
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_LLH_TRANSFORMER_HPP__
