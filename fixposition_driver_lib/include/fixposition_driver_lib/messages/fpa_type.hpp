/**
 *  @file
 *  @brief Declaration of FP_A type messages
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

#ifndef __FIXPOSITION_DRIVER_LIB_CONVERTER_FPA_TYPE__
#define __FIXPOSITION_DRIVER_LIB_CONVERTER_FPA_TYPE__

/* PACKAGE */
#include <fixposition_driver_lib/messages/msg_data.hpp>
#include <fixposition_driver_lib/time_conversions.hpp>

namespace fixposition {

// ------------ FP_A-ODOMETRY ------------

struct FP_ODOMETRY {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Message fields
    OdometryData odom;
    Eigen::Vector3d acceleration;
    int fusion_status;
    int imu_bias_status;
    int gnss1_status;
    int gnss2_status;
    int wheelspeed_status;
    std::string version;
    
    // Message structure
    const std::string frame_id = "FP_ECEF";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "ODOMETRY";
    static constexpr unsigned int kVersion_ = 2;
    static constexpr unsigned int kSize_ = 45;

    FP_ODOMETRY() {
        odom.stamp = fixposition::times::GpsTime();
        odom.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;
        odom.pose = PoseWithCovData();
        odom.twist = TwistWithCovData();
        acceleration.setZero();
        fusion_status = -1;
        imu_bias_status = -1;
        gnss1_status = -1;
        gnss2_status = -1;
        wheelspeed_status = -1;
        version = "Unknown";
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        odom.stamp = fixposition::times::GpsTime();
        odom.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;
        odom.pose = PoseWithCovData();
        odom.twist = TwistWithCovData();
        acceleration.setZero();
        fusion_status = -1;
        imu_bias_status = -1;
        gnss1_status = -1;
        gnss2_status = -1;
        wheelspeed_status = -1;
        version = "Unknown";
    }
};

// ------------ FP_A-ODOMENU ------------

struct FP_ODOMENU {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Message fields
    OdometryData odom;
    Eigen::Vector3d acceleration;
    int fusion_status;
    int imu_bias_status;
    int gnss1_status;
    int gnss2_status;
    int wheelspeed_status;

    // Message structure
    const std::string frame_id = "FP_ENU0";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "ODOMENU";
    static constexpr unsigned int kVersion_ = 1;
    static constexpr unsigned int kSize_ = 44;

    FP_ODOMENU() {
        odom.stamp = fixposition::times::GpsTime();
        odom.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;
        odom.pose = PoseWithCovData();
        odom.twist = TwistWithCovData();
        acceleration.setZero();
        fusion_status = -1;
        imu_bias_status = -1;
        gnss1_status = -1;
        gnss2_status = -1;
        wheelspeed_status = -1;
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        odom.stamp = fixposition::times::GpsTime();
        odom.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;
        odom.pose = PoseWithCovData();
        odom.twist = TwistWithCovData();
        acceleration.setZero();
        fusion_status = -1;
        imu_bias_status = -1;
        gnss1_status = -1;
        gnss2_status = -1;
        wheelspeed_status = -1;
    }
};

// ------------ FP_A-ODOMSH ------------

struct FP_ODOMSH {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Message fields
    OdometryData odom;
    Eigen::Vector3d acceleration;
    int fusion_status;
    int imu_bias_status;
    int gnss1_status;
    int gnss2_status;
    int wheelspeed_status;

    // Message structure
    const std::string frame_id = "FP_ECEF";
    const std::string child_frame_id = "FP_POISH";
    const std::string header_ = "ODOMSH";
    static constexpr unsigned int kVersion_ = 1;
    static constexpr unsigned int kSize_ = 44;

    FP_ODOMSH() {
        odom.stamp = fixposition::times::GpsTime();
        odom.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;
        odom.pose = PoseWithCovData();
        odom.twist = TwistWithCovData();
        acceleration.setZero();
        fusion_status = -1;
        imu_bias_status = -1;
        gnss1_status = -1;
        gnss2_status = -1;
        wheelspeed_status = -1;
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        odom.stamp = fixposition::times::GpsTime();
        odom.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;
        odom.pose = PoseWithCovData();
        odom.twist = TwistWithCovData();
        acceleration.setZero();
        fusion_status = -1;
        imu_bias_status = -1;
        gnss1_status = -1;
        gnss2_status = -1;
        wheelspeed_status = -1;
    }
};

// ------------ FP_A-LLH ------------

struct FP_LLH {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Message fields
    times::GpsTime stamp;    
    Eigen::Vector3d llh;
    Eigen::Matrix<double, 3, 3> cov;

    // Message structure
    const std::string frame_id = "FP_LLH";
    const std::string child_frame_id = "FP_POI";
    const std::string header_ = "LLH";
    static constexpr unsigned int kVersion_ = 1;
    static constexpr unsigned int kSize_ = 14;

    FP_LLH() {
        stamp = fixposition::times::GpsTime();
        llh.setZero();
        cov.setZero();
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        stamp = fixposition::times::GpsTime();
        llh.setZero();
        cov.setZero();
    }
};

// ------------ FP_A-TF ------------

struct FP_TF {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Message fields
    TfData tf;
    bool valid_tf;

    // Message structure
    const std::string header_ = "TF";
    static constexpr unsigned int kVersion_ = 2;
    static constexpr unsigned int kSize_ = 14;

    FP_TF() {
        tf.stamp = fixposition::times::GpsTime();
        tf.frame_id = "";
        tf.child_frame_id = "";
        tf.translation.setZero();
        tf.rotation.setIdentity();
        valid_tf = false;
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        tf.stamp = fixposition::times::GpsTime();
        tf.frame_id = "";
        tf.child_frame_id = "";
        tf.translation.setZero();
        tf.rotation.setIdentity();
        valid_tf = false;
    }
};

// ------------ FP_A-RAWIMU ------------

struct FP_RAWIMU {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Message fields
    ImuData imu;

    // Message structure
    const std::string header_ = "RAWIMU";
    static constexpr unsigned int kVersion_ = 1;
    static constexpr unsigned int kSize_ = 11;

    FP_RAWIMU() {
        imu.stamp = fixposition::times::GpsTime();
        imu.frame_id = "";
        imu.linear_acceleration.setZero();
        imu.angular_velocity.setZero();
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        imu.stamp = fixposition::times::GpsTime();
        imu.frame_id = "";
        imu.linear_acceleration.setZero();
        imu.angular_velocity.setZero();
    }
};

// ------------ FP_A-CORRIMU ------------

struct FP_CORRIMU {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Message fields
    ImuData imu;

    // Message structure
    const std::string header_ = "CORRIMU";
    static constexpr unsigned int kVersion_ = 1;
    static constexpr unsigned int kSize_ = 11;

    FP_CORRIMU() {
        imu.stamp = fixposition::times::GpsTime();
        imu.frame_id = "";
        imu.linear_acceleration.setZero();
        imu.angular_velocity.setZero();
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        imu.stamp = fixposition::times::GpsTime();
        imu.frame_id = "";
        imu.linear_acceleration.setZero();
        imu.angular_velocity.setZero();
    }
};

// ------------ FP_A-GNSSANT ------------

struct FP_GNSSANT {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Message fields
    times::GpsTime stamp;
    std::string gnss1_state;
    std::string gnss1_power;
    int gnss1_age;
    std::string gnss2_state;
    std::string gnss2_power;
    int gnss2_age;

    // Message structure
    const std::string header_ = "GNSSANT";
    static constexpr unsigned int kVersion_ = 1;
    static constexpr unsigned int kSize_ = 11;

    FP_GNSSANT() {
        stamp = fixposition::times::GpsTime();
        gnss1_state = "";
        gnss1_power = "";
        gnss1_age = -1;
        gnss2_state = "";
        gnss2_power = "";
        gnss2_age = -1;
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        stamp = fixposition::times::GpsTime();
        gnss1_state = "";
        gnss1_power = "";
        gnss1_age = -1;
        gnss2_state = "";
        gnss2_power = "";
        gnss2_age = -1;
    }
};

// ------------ FP_A-GNSSCORR ------------

struct FP_GNSSCORR {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Message fields
    times::GpsTime stamp;
    int gnss1_fix;
    int gnss1_nsig_l1;
    int gnss1_nsig_l2;
    int gnss2_fix;
    int gnss2_nsig_l1;
    int gnss2_nsig_l2;
    float corr_latency;
    float corr_update_rate;
    float corr_data_rate;
    float corr_msg_rate;
    int sta_id;
    Eigen::Vector3d sta_llh;
    int sta_dist;

    // Message structure
    const std::string header_ = "GNSSCORR";
    static constexpr unsigned int kVersion_ = 1;
    static constexpr unsigned int kSize_ = 20;

    FP_GNSSCORR() {
        stamp = fixposition::times::GpsTime();
        gnss1_fix = -1;
        gnss1_nsig_l1 = -1;
        gnss1_nsig_l2 = -1;
        gnss2_fix = -1;
        gnss2_nsig_l1 = -1;
        gnss2_nsig_l2 = -1;
        corr_latency = -1.0;
        corr_update_rate = -1.0;
        corr_data_rate = -1.0;
        corr_msg_rate = -1.0;
        sta_id = -1;
        sta_llh.setZero();
        sta_dist = -1;
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        stamp = fixposition::times::GpsTime();
        gnss1_fix = -1;
        gnss1_nsig_l1 = -1;
        gnss1_nsig_l2 = -1;
        gnss2_fix = -1;
        gnss2_nsig_l1 = -1;
        gnss2_nsig_l2 = -1;
        corr_latency = -1.0;
        corr_update_rate = -1.0;
        corr_data_rate = -1.0;
        corr_msg_rate = -1.0;
        sta_id = -1;
        sta_llh.setZero();
        sta_dist = -1;
    }
};

// ------------ FP_A-TEXT ------------

struct FP_TEXT {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Message fields
    std::string level;
    std::string text;

    // Message structure
    const std::string header_ = "TEXT";
    static constexpr unsigned int kVersion_ = 1;
    static constexpr unsigned int kSize_ = 5;

    FP_TEXT() {
        level = "";
        text = "";
    }

    void ConvertFromTokens(const std::vector<std::string>& tokens);

    void ResetData() {
        level = "";
        text = "";
    }
};

}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CONVERTER_FPA_TYPE__
