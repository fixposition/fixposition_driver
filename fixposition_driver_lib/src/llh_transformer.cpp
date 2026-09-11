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

/* LIBC/STL */
#include <cmath>
#include <limits>
#include <string>

/* EXTERNAL */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/math.hpp>
#include <fpsdk_common/trafo.hpp>
#if FIXPOSITION_DRIVER_USE_PROJ
#include <proj.h>
#endif

/* PACKAGE */
#include "fixposition_driver_lib/llh_transformer.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

#if FIXPOSITION_DRIVER_USE_PROJ
namespace {

std::string ProjError(PJ_CONTEXT* context, const std::string& fallback) {
    const int error = proj_context_errno(context);
    const char* message = proj_context_errno_string(context, error);
    return ((message != nullptr) && (message[0] != '\0')) ? std::string(message) : fallback;
}

bool IsSupportedTarget(PJ_CONTEXT* context, const PJ* target) {
    const PJ_TYPE type = proj_get_type(target);
    if (type == PJ_TYPE_GEOGRAPHIC_3D_CRS) {
        return true;
    }
    if (type != PJ_TYPE_COMPOUND_CRS) {
        return false;
    }

    PJ* horizontal = proj_crs_get_sub_crs(context, target, 0);
    PJ* vertical = proj_crs_get_sub_crs(context, target, 1);
    const bool supported = (horizontal != nullptr) && (vertical != nullptr) &&
                           (proj_get_type(horizontal) == PJ_TYPE_GEOGRAPHIC_2D_CRS) &&
                           (proj_get_type(vertical) == PJ_TYPE_VERTICAL_CRS);
    proj_destroy(horizontal);
    proj_destroy(vertical);
    return supported;
}

class ThreadProjTransformer {
   public:
    ThreadProjTransformer() = default;
    ~ThreadProjTransformer() { Reset(); }

    bool Configure(const LlhTransformer* owner, const std::string& ecef_crs, const std::string& llh_crs,
                   std::string& error) {
        if ((owner_ == owner) && (ecef_crs_ == ecef_crs) && (llh_crs_ == llh_crs)) {
            error = error_;
            return transform_ != nullptr;
        }

        Reset();
        owner_ = owner;
        ecef_crs_ = ecef_crs;
        llh_crs_ = llh_crs;
        context_ = proj_context_create();
        if (context_ == nullptr) {
            error_ = "failed creating PROJ context";
            error = error_;
            return false;
        }
        proj_context_set_enable_network(context_, false);

        PJ* source = proj_create(context_, ecef_crs.c_str());
        if (source == nullptr) {
            error_ = "invalid source CRS: " + ProjError(context_, "unknown PROJ error");
            error = error_;
            return false;
        }
        PJ* target = proj_create(context_, llh_crs.c_str());
        if (target == nullptr) {
            error_ = "invalid target CRS: " + ProjError(context_, "unknown PROJ error");
            proj_destroy(source);
            error = error_;
            return false;
        }

        if (proj_get_type(source) != PJ_TYPE_GEOCENTRIC_CRS) {
            error_ = "source CRS must be geocentric";
        } else if (!IsSupportedTarget(context_, target)) {
            error_ = "target CRS must be geographic 3D or geographic 2D plus vertical";
        } else {
            const char* options[] = {"ALLOW_BALLPARK=NO", "ONLY_BEST=YES", nullptr};
            PJ* operation = proj_create_crs_to_crs_from_pj(context_, source, target, nullptr, options);
            if (operation == nullptr) {
                error_ = "failed creating coordinate operation: " + ProjError(context_, "unknown PROJ error");
            } else if (!proj_coordoperation_is_instantiable(context_, operation)) {
                error_ = "coordinate operation is unavailable; check that all required PROJ grids are installed";
                proj_destroy(operation);
            } else {
                // Normalized output has longitude, latitude and height order regardless of the CRS's native axis order.
                transform_ = proj_normalize_for_visualization(context_, operation);
                proj_destroy(operation);
                if (transform_ == nullptr) {
                    error_ = "failed normalizing coordinate operation: " + ProjError(context_, "unknown PROJ error");
                }
            }
        }

        proj_destroy(source);
        proj_destroy(target);
        error = error_;
        return transform_ != nullptr;
    }

    bool Transform(const Eigen::Vector3d& ecef, Eigen::Vector3d& llh_rad, std::string& error) {
        if (transform_ == nullptr) {
            error = error_;
            return false;
        }

        proj_errno_reset(transform_);
        const PJ_COORD input = proj_coord(ecef.x(), ecef.y(), ecef.z(), std::numeric_limits<double>::infinity());
        const PJ_COORD output = proj_trans(transform_, PJ_FWD, input);
        const int transform_error = proj_errno(transform_);
        if ((transform_error != 0) || !std::isfinite(output.xyz.x) || !std::isfinite(output.xyz.y) ||
            !std::isfinite(output.xyz.z)) {
            const char* message = proj_errno_string(transform_error);
            error = ((message != nullptr) && (message[0] != '\0')) ? message : "coordinate transformation failed";
            return false;
        }

        llh_rad = {fpsdk::common::math::DegToRad(output.xyz.y), fpsdk::common::math::DegToRad(output.xyz.x),
                   output.xyz.z};
        return true;
    }

    bool warning_reported() const { return warning_reported_; }
    void set_warning_reported() { warning_reported_ = true; }

   private:
    void Reset() {
        if (transform_ != nullptr) {
            proj_destroy(transform_);
        }
        if (context_ != nullptr) {
            proj_context_destroy(context_);
        }
        transform_ = nullptr;
        context_ = nullptr;
        owner_ = nullptr;
        ecef_crs_.clear();
        llh_crs_.clear();
        error_.clear();
        warning_reported_ = false;
    }

    const LlhTransformer* owner_ = nullptr;
    std::string ecef_crs_;
    std::string llh_crs_;
    std::string error_;
    PJ_CONTEXT* context_ = nullptr;
    PJ* transform_ = nullptr;
    bool warning_reported_ = false;
};

}  // namespace
#endif  // FIXPOSITION_DRIVER_USE_PROJ

bool LlhTransformer::Init(bool enabled, const std::string& ecef_crs, const std::string& llh_crs) {
    proj_enabled_ = false;
    ecef_crs_ = ecef_crs;
    llh_crs_ = llh_crs;
    error_.clear();
    if (!enabled) {
        return true;
    }

#if FIXPOSITION_DRIVER_USE_PROJ
    // Validate using temporary state so the context is created and destroyed by this thread. Transforming threads
    // create their own cached state on first use.
    ThreadProjTransformer validator;
    if (!validator.Configure(this, ecef_crs_, llh_crs_, error_)) {
        return false;
    }
    proj_enabled_ = true;
    return true;
#else
    error_ = "driver was built without PROJ support";
    return false;
#endif
}

bool LlhTransformer::EcefToLlhRad(const Eigen::Vector3d& ecef, Eigen::Vector3d& llh_rad) const {
    if (!proj_enabled_) {
        llh_rad = fpsdk::common::trafo::TfWgs84LlhEcef(ecef);
        return true;
    }

#if FIXPOSITION_DRIVER_USE_PROJ
    thread_local ThreadProjTransformer transformer;
    std::string error;
    if (!transformer.Configure(this, ecef_crs_, llh_crs_, error) || !transformer.Transform(ecef, llh_rad, error)) {
        if (!transformer.warning_reported()) {
            WARNING("PROJ ECEF to LLH transformation failed: %s", error.c_str());
            transformer.set_warning_reported();
        }
        return false;
    }
    return true;
#else
    (void)ecef;
    (void)llh_rad;
    return false;
#endif
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
