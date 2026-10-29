/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 */

/* LIBC/STL */
#include <memory>
#include <string>
#include <thread>

/* EXTERNAL */
#include <gtest/gtest.h>

#include <fpsdk_common/math.hpp>
#include <fpsdk_common/trafo.hpp>

/* PACKAGE */
#include "fixposition_driver_lib/helper.hpp"
#include "fixposition_driver_lib/params.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

TEST(LlhTrafo, ParamsUseWgs84ThreeDimensionalDefaults) {
    const DriverParams params;
    EXPECT_FALSE(params.datum_llh_enabled_);
    EXPECT_EQ(params.datum_llh_ecef_crs_, "EPSG:4978");
    EXPECT_EQ(params.datum_llh_llh_crs_, "EPSG:4979");
}

TEST(OdometryData, EnuOriginAlwaysUsesEllipsoidalWgs84Height) {
    const Eigen::Vector3d llh_ref(fpsdk::common::math::DegToRad(47.0), fpsdk::common::math::DegToRad(8.0), 500.0);
    const Eigen::Vector3d ecef_ref = fpsdk::common::trafo::TfEcefWgs84Llh(llh_ref);

    TfData tf;
    tf.valid = true;
    tf.translation = ecef_ref;
    tf.rotation.setIdentity();

    OdometryData odometry;
    odometry.pose.position = ecef_ref;
    odometry.pose.orientation.setIdentity();
    ASSERT_TRUE(odometry.ConvertToEnu(tf));
    EXPECT_LT(odometry.pose.position.norm(), 1e-3) << odometry.pose.position.transpose();
}

#if FPSDK_USE_PROJ
using fpsdk::common::trafo::Transformer;

// Driver params with datum LLH enabled
static DriverParams LlhParams(const std::string& ecef_crs, const std::string& llh_crs) {
    DriverParams params;
    params.datum_llh_enabled_ = true;
    params.datum_llh_ecef_crs_ = ecef_crs;
    params.datum_llh_llh_crs_ = llh_crs;
    return params;
}

TEST(LlhTrafo, DisabledUsesBuiltInWgs84Conversion) {
    DriverParams params;
    params.datum_llh_ecef_crs_ = "not a CRS";
    params.datum_llh_llh_crs_ = "also not a CRS";
    std::unique_ptr<Transformer> llh_trafo;
    std::string error;
    ASSERT_TRUE(InitLlhTrafo(params, llh_trafo, error));
    EXPECT_FALSE(llh_trafo);
    EXPECT_TRUE(error.empty());

    const Eigen::Vector3d expected_llh(fpsdk::common::math::DegToRad(47.0), fpsdk::common::math::DegToRad(8.0), 500.0);
    const Eigen::Vector3d ecef = fpsdk::common::trafo::TfEcefWgs84Llh(expected_llh);
    Eigen::Vector3d actual_llh;
    ASSERT_TRUE(EcefToLlhRad(llh_trafo.get(), ecef, actual_llh));
    EXPECT_TRUE(actual_llh.isApprox(expected_llh, 1e-8));
}

TEST(LlhTrafo, RejectsInvalidAndNonGeographicCrs) {
    std::unique_ptr<Transformer> llh_trafo;
    std::string error;
    EXPECT_FALSE(InitLlhTrafo(LlhParams("EPSG:4978", "EPSG:4979+3855"), llh_trafo, error));
    EXPECT_FALSE(llh_trafo);
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(InitLlhTrafo(LlhParams("EPSG:4978", "EPSG:2056"), llh_trafo, error));
    EXPECT_FALSE(llh_trafo);
    EXPECT_EQ(error, "target CRS must be geographic 3D or geographic 2D plus vertical");
    EXPECT_FALSE(InitLlhTrafo(LlhParams("EPSG:4326", "EPSG:4979"), llh_trafo, error));
    EXPECT_FALSE(llh_trafo);
    EXPECT_EQ(error, "source CRS must be geocentric");
}

TEST(LlhTrafo, MatchesBuiltInWgs84Conversion) {
    std::unique_ptr<Transformer> llh_trafo;
    std::string error;
    ASSERT_TRUE(InitLlhTrafo(LlhParams("EPSG:4978", "EPSG:4979"), llh_trafo, error)) << error;
    ASSERT_TRUE(llh_trafo);

    // Checks lon/lat swap and deg to rad
    const Eigen::Vector3d expected_llh(fpsdk::common::math::DegToRad(47.0), fpsdk::common::math::DegToRad(8.0), 500.0);
    const Eigen::Vector3d ecef = fpsdk::common::trafo::TfEcefWgs84Llh(expected_llh);
    Eigen::Vector3d actual_llh;
    ASSERT_TRUE(EcefToLlhRad(llh_trafo.get(), ecef, actual_llh));
    EXPECT_TRUE(actual_llh.isApprox(expected_llh, 1e-8)) << actual_llh.transpose();
}

TEST(LlhTrafo, TransformsOnAThreadDifferentFromInitialization) {
    std::unique_ptr<Transformer> llh_trafo;
    std::string error;
    ASSERT_TRUE(InitLlhTrafo(LlhParams("EPSG:4978", "EPSG:4979"), llh_trafo, error)) << error;

    const Eigen::Vector3d expected_llh(fpsdk::common::math::DegToRad(47.0), fpsdk::common::math::DegToRad(8.0), 500.0);
    const Eigen::Vector3d ecef = fpsdk::common::trafo::TfEcefWgs84Llh(expected_llh);
    Eigen::Vector3d actual_llh;
    bool success = false;
    std::thread worker([&]() { success = EcefToLlhRad(llh_trafo.get(), ecef, actual_llh); });
    worker.join();

    ASSERT_TRUE(success);
    EXPECT_TRUE(actual_llh.isApprox(expected_llh, 1e-8));
}
#endif

/* ****************************************************************************************************************** */
}  // namespace fixposition
