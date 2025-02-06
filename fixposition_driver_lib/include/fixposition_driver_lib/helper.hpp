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
 * @brief Helper functions and types
 */

#ifndef __FIXPOSITION_DRIVER_LIB_HELPER_HPP__
#define __FIXPOSITION_DRIVER_LIB_HELPER_HPP__

/* LIBC/STL */
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */
#include <fpsdk_common/ext/eigen_core.hpp>
#include <fpsdk_common/ext/eigen_geometry.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/fpb.hpp>
#include <fpsdk_common/parser/nmea.hpp>
#include <fpsdk_common/parser/novb.hpp>
#include <fpsdk_common/time.hpp>

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

static constexpr const char* ODOMETRY_FRAME_ID = "FP_ECEF";
static constexpr const char* ODOMETRY_CHILD_FRAME_ID = "FP_POI";

static constexpr const char* ODOMENU_FRAME_ID = "FP_ENU0";
static constexpr const char* ODOMENU_CHILD_FRAME_ID = "FP_POI";

static constexpr const char* ODOMSH_FRAME_ID = "FP_ECEF";
static constexpr const char* ODOMSH_CHILD_FRAME_ID = "FP_POISH";

static constexpr const char* IMU_FRAME_ID = "FP_VRTK";

static constexpr const char* GNSS_FRAME_ID = "GNSS";
static constexpr const char* GNSS1_FRAME_ID = "GNSS1";
static constexpr const char* GNSS2_FRAME_ID = "GNSS2";

static constexpr const char* ENU_FRAME_ID = "FP_ENU";

fpsdk::common::time::Time FpaGpsTimeToTime(const fpsdk::common::parser::fpa::FpaGpsTime gps_time);

struct TfData {
    bool valid = false;
    fpsdk::common::time::Time stamp;
    std::string frame_id;
    std::string child_frame_id;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    TfData() {
        translation.setZero();
        rotation.setIdentity();
    }
    bool SetFromFpaTfPayload(const fpsdk::common::parser::fpa::FpaTfPayload& payload);
};

struct PoseWithCovData {
    bool valid = false;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Matrix<double, 6, 6> cov;
    PoseWithCovData() {
        position.setZero();
        orientation.setIdentity();
        cov.setZero();
    }
    bool SetFromFpaOdomPayload(const fpsdk::common::parser::fpa::FpaOdomPayload& payload);
};

struct TwistWithCovData {
    bool valid = false;
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
    Eigen::Matrix<double, 6, 6> cov;
    TwistWithCovData() {
        linear.setZero();
        angular.setZero();
        cov.setZero();
    }
    bool SetFromFpaOdomPayload(const fpsdk::common::parser::fpa::FpaOdomPayload& payload);
};

struct OdometryData {
    bool valid = false;
    enum class Type { UNSPECIFIED, ODOMETRY, ODOMENU, ODOMSH };
    Type type = Type::UNSPECIFIED;
    fpsdk::common::time::Time stamp;
    std::string frame_id;
    std::string child_frame_id;
    PoseWithCovData pose;
    TwistWithCovData twist;
    bool SetFromFpaOdomPayload(const fpsdk::common::parser::fpa::FpaOdomPayload& payload);
};

/**
 * @brief Build a 3x3 covariance matrix
 *
 *    | xx xy xz |
 *    | xy yy yz |
 *    | xz yz zz |
 */
inline Eigen::Matrix<double, 3, 3> BuildCovMat3D(const double xx, const double yy, const double zz, const double xy,
                                                 const double yz, const double xz) {
    Eigen::Matrix<double, 3, 3> cov;
    cov.setZero();

    // Diagonals
    cov(0, 0) = xx;  // 0
    cov(1, 1) = yy;  // 4
    cov(2, 2) = zz;  // 8

    // Rest of values
    cov(1, 0) = cov(0, 1) = xy;  // 1 = 3
    cov(2, 1) = cov(1, 2) = yz;  // 2 = 6
    cov(2, 0) = cov(0, 2) = xz;  // 5 = 7

    return cov;
}

/**
 * @brief Build a 6x6 covariance matrix which is 2 independent 3x3 matrices
 *
 *    | xx  xy  xz   0    0    0  |
 *    | xy  yy  yz   0    0    0  |
 *    | xz  yz  zz   0    0    0  |
 *    |  0   0   0  xx1  xy1  xz1 |
 *    |  0   0   0  xy1  yy1  yz1 |
 *    |  0   0   0  xz1  yz1  zz1 |
 */
inline Eigen::Matrix<double, 6, 6> BuildCovMat6D(const double xx, const double yy, const double zz, const double xy,
                                                 const double yz, const double xz, double xx1, const double yy1,
                                                 const double zz1, const double xy1, const double yz1, double xz1) {
    Eigen::Matrix<double, 6, 6> cov;
    cov.setZero();

    // Diagonals
    cov(0, 0) = xx;   // 0
    cov(1, 1) = yy;   // 7
    cov(2, 2) = zz;   // 14
    cov(3, 3) = xx1;  // 21
    cov(4, 4) = yy1;  // 28
    cov(5, 5) = zz1;  // 35

    // Rest of values
    cov(1, 0) = cov(0, 1) = xy;   // 1 = 6
    cov(2, 1) = cov(1, 2) = yz;   // 8 = 13
    cov(2, 0) = cov(0, 2) = xz;   // 2 = 12
    cov(4, 3) = cov(3, 4) = xy1;  // 22 = 27
    cov(5, 4) = cov(4, 5) = yz1;  // 29 = 34
    cov(5, 3) = cov(3, 5) = xz1;  // 23 = 33

    return cov;
}

struct JumpDetector {
    JumpDetector();
    bool Check(const OdometryData& odometry_data);

    Eigen::Vector3d curr_pos_;
    Eigen::MatrixXd curr_cov_;
    fpsdk::common::time::Time curr_stamp_;
    Eigen::Vector3d prev_pos_;
    Eigen::MatrixXd prev_cov_;
    fpsdk::common::time::Time prev_stamp_;
    Eigen::Vector3d pos_diff_;
    std::string warning_;
};

struct WheelSpeedData {
    std::string location_;
    bool vx_valid_ = false;
    int32_t vx_ = 0;
    bool vy_valid_ = false;
    int32_t vy_ = 0;
    bool vz_valid_ = false;
    int32_t vz_ = 0;
};

template <typename RosMsgT>  // e.g. fixposition_driver_msgs::Speed
inline std::vector<WheelSpeedData> SpeedMsgToWheelspeedData(const RosMsgT& msg) {
    std::vector<WheelSpeedData> data;
    for (const auto& sensor : msg.sensors) {
        data.push_back(
            {sensor.location, sensor.vx_valid, sensor.vx, sensor.vy_valid, sensor.vy, sensor.vz_valid, sensor.vz});
    }
    return data;
}

struct NmeaEpochData {
    NmeaEpochData(const fpsdk::common::parser::fpa::FpaEpoch epoch);

    // Collectors
    fpsdk::common::parser::fpa::FpaEpoch epoch_;
    fpsdk::common::parser::nmea::NmeaGgaPayload gga_;
    fpsdk::common::parser::nmea::NmeaGllPayload gll_;
    fpsdk::common::parser::nmea::NmeaGsaPayload gsa_;
    fpsdk::common::parser::nmea::NmeaGstPayload gst_;
    fpsdk::common::parser::nmea::NmeaHdtPayload hdt_;
    fpsdk::common::parser::nmea::NmeaRmcPayload rmc_;
    fpsdk::common::parser::nmea::NmeaVtgPayload vtg_;
    fpsdk::common::parser::nmea::NmeaZdaPayload zda_;
    fpsdk::common::parser::nmea::NmeaCollectGsaGsv gsa_gsv_;

    // "Best of" data, populated by Complete(), NmeaEpoch.msg for docu
    fpsdk::common::time::Time stamp_;
    std::string frame_id_;
    fpsdk::common::parser::nmea::NmeaDate date_;
    fpsdk::common::parser::nmea::NmeaTime time_;
    fpsdk::common::parser::nmea::NmeaStatusGllRmc status_ = fpsdk::common::parser::nmea::NmeaStatusGllRmc::UNSPECIFIED;
    fpsdk::common::parser::nmea::NmeaNavStatusRmc navstatus_ =
        fpsdk::common::parser::nmea::NmeaNavStatusRmc::UNSPECIFIED;
    fpsdk::common::parser::nmea::NmeaModeRmcGns mode1_ = fpsdk::common::parser::nmea::NmeaModeRmcGns::UNSPECIFIED;
    fpsdk::common::parser::nmea::NmeaModeGllVtg mode2_ = fpsdk::common::parser::nmea::NmeaModeGllVtg::UNSPECIFIED;
    fpsdk::common::parser::nmea::NmeaQualityGga quality_ = fpsdk::common::parser::nmea::NmeaQualityGga::UNSPECIFIED;
    fpsdk::common::parser::nmea::NmeaOpModeGsa opmode_ = fpsdk::common::parser::nmea::NmeaOpModeGsa::UNSPECIFIED;
    fpsdk::common::parser::nmea::NmeaNavModeGsa navmode_ = fpsdk::common::parser::nmea::NmeaNavModeGsa::UNSPECIFIED;
    fpsdk::common::parser::nmea::NmeaLlh llh_;
    fpsdk::common::parser::nmea::NmeaInt num_sv_;
    fpsdk::common::parser::nmea::NmeaFloat rms_range_;
    fpsdk::common::parser::nmea::NmeaFloat std_major_;
    fpsdk::common::parser::nmea::NmeaFloat std_minor_;
    fpsdk::common::parser::nmea::NmeaFloat angle_major_;
    fpsdk::common::parser::nmea::NmeaFloat std_lat_;
    fpsdk::common::parser::nmea::NmeaFloat std_lon_;
    fpsdk::common::parser::nmea::NmeaFloat std_alt_;
    fpsdk::common::parser::nmea::NmeaFloat pdop_;
    fpsdk::common::parser::nmea::NmeaFloat hdop_;
    fpsdk::common::parser::nmea::NmeaFloat vdop_;
    fpsdk::common::parser::nmea::NmeaFloat heading_;
    fpsdk::common::parser::nmea::NmeaFloat speed_;
    fpsdk::common::parser::nmea::NmeaFloat course_;
    fpsdk::common::parser::nmea::NmeaFloat cogt_;
    fpsdk::common::parser::nmea::NmeaFloat cogm_;
    fpsdk::common::parser::nmea::NmeaFloat sogn_;
    fpsdk::common::parser::nmea::NmeaFloat sogk_;
    fpsdk::common::parser::nmea::NmeaFloat diff_age_;
    fpsdk::common::parser::nmea::NmeaInt diff_sta_;
    fpsdk::common::parser::nmea::NmeaInt local_hr_;
    fpsdk::common::parser::nmea::NmeaInt local_min_;

    Eigen::Matrix<double, 3, 3> cov_enu_;

    // Completes the data and returns it, and resets the instance back to empty
    NmeaEpochData CompleteAndReset();
};

// Fusion epoch data, see FusionEpoch.msg for docu
struct FusionEpochData {
    FusionEpochData();

    bool fpa_odometry_avail_ = false;
    fpsdk::common::parser::fpa::FpaOdometryPayload fpa_odometry_;
    void CollectFpaOdometry(const fpsdk::common::parser::fpa::FpaOdometryPayload& payload);

    bool fpa_odomsh_avail_ = false;
    fpsdk::common::parser::fpa::FpaOdomshPayload fpa_odomsh_;
    void CollectFpaOdomsh(const fpsdk::common::parser::fpa::FpaOdomshPayload& payload);

    bool fpa_odomenu_avail_ = false;
    fpsdk::common::parser::fpa::FpaOdomenuPayload fpa_odomenu_;
    void CollectFpaOdomenu(const fpsdk::common::parser::fpa::FpaOdomenuPayload& payload);

    bool fpa_odomstatus_avail_ = false;
    fpsdk::common::parser::fpa::FpaOdomstatusPayload fpa_odomstatus_;
    void CollectFpaOdomstatus(const fpsdk::common::parser::fpa::FpaOdomstatusPayload& payload);

    bool novb_inspvax_avail_ = false;
    fpsdk::common::parser::novb::NovbHeader novb_inspvax_header_;
    fpsdk::common::parser::novb::NovbInspvax novb_inspvax_payload_;
    void CollectNovbInspvax(const fpsdk::common::parser::novb::NovbHeader* header,
                            const fpsdk::common::parser::novb::NovbInspvax* payload);

    bool fpa_eoe_avail_ = false;
    fpsdk::common::parser::fpa::FpaEoePayload fpa_eoe_;

    bool fpa_imubias_avail_ = false;
    fpsdk::common::parser::fpa::FpaImubiasPayload fpa_imubias_;
    void CollectFpaImubias(const fpsdk::common::parser::fpa::FpaImubiasPayload& payload);

    // Completes the data and returns it, and sets up the instance for the next epoch
    FusionEpochData CompleteAndReset(const fpsdk::common::parser::fpa::FpaEoePayload& eoe);
};

void HelloWorld();

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_HELPER_HPP__
