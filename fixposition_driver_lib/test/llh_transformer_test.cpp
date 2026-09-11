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
#include <thread>

/* EXTERNAL */
#include <gtest/gtest.h>

#include <fpsdk_common/math.hpp>
#include <fpsdk_common/trafo.hpp>

/* PACKAGE */
#include "fixposition_driver_lib/helper.hpp"
#include "fixposition_driver_lib/llh_transformer.hpp"
#include "fixposition_driver_lib/params.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

TEST(LlhTransformer, ParamsUseWgs84ThreeDimensionalDefaults) {
    const DriverParams params;
    EXPECT_EQ(params.datum_llh_ecef_crs_, "EPSG:4978");
    EXPECT_EQ(params.datum_llh_llh_crs_, "EPSG:4979");
}

TEST(LlhTransformer, DisabledUsesBuiltInWgs84Conversion) {
    LlhTransformer transformer;
    ASSERT_TRUE(transformer.Init(false, "not a CRS", "also not a CRS"));
    EXPECT_FALSE(transformer.enabled());

    const Eigen::Vector3d expected_llh(fpsdk::common::math::DegToRad(47.0), fpsdk::common::math::DegToRad(8.0), 500.0);
    const Eigen::Vector3d ecef = fpsdk::common::trafo::TfEcefWgs84Llh(expected_llh);
    Eigen::Vector3d actual_llh;
    ASSERT_TRUE(transformer.EcefToLlhRad(ecef, actual_llh));
    EXPECT_TRUE(actual_llh.isApprox(expected_llh, 1e-8));
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

#if FIXPOSITION_DRIVER_USE_PROJ
TEST(LlhTransformer, RejectsInvalidAndNonGeographicCrs) {
    LlhTransformer transformer;
    EXPECT_FALSE(transformer.Init(true, "EPSG:4978", "EPSG:4979+3855"));
    EXPECT_FALSE(transformer.Init(true, "EPSG:4978", "EPSG:2056"));
    EXPECT_FALSE(transformer.Init(true, "EPSG:4326", "EPSG:4979"));
}

TEST(LlhTransformer, TransformsOnAThreadDifferentFromInitialization) {
    LlhTransformer transformer;
    ASSERT_TRUE(transformer.Init(true, "EPSG:4978", "EPSG:4979")) << transformer.error();

    const Eigen::Vector3d expected_llh(fpsdk::common::math::DegToRad(47.0), fpsdk::common::math::DegToRad(8.0), 500.0);
    const Eigen::Vector3d ecef = fpsdk::common::trafo::TfEcefWgs84Llh(expected_llh);
    Eigen::Vector3d actual_llh;
    bool success = false;
    std::thread worker([&]() { success = transformer.EcefToLlhRad(ecef, actual_llh); });
    worker.join();

    ASSERT_TRUE(success);
    EXPECT_TRUE(actual_llh.isApprox(expected_llh, 1e-8));
}
#else
TEST(LlhTransformer, EnabledFailsWhenProjWasCompiledOut) {
    LlhTransformer transformer;
    EXPECT_FALSE(transformer.Init(true, "EPSG:4978", "EPSG:4979"));
    EXPECT_FALSE(transformer.enabled());
    EXPECT_FALSE(transformer.error().empty());
}
#endif

/* ****************************************************************************************************************** */
}  // namespace fixposition
