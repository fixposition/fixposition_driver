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
 * @brief Convert data to ROS2 msgs
 */

/* LIBC/STL */

/* EXTERNAL */
#include <fixposition_driver_msgs/data_to_ros.hpp>
#include <fpsdk_common/math.hpp>
#include <fpsdk_common/time.hpp>
#include <fpsdk_common/trafo.hpp>
#include <fpsdk_ros2/utils.hpp>

/* PACKAGE */
#include "fixposition_driver_ros2/data_to_ros2.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk;
using namespace fpsdk::common;
using namespace fpsdk::common::parser;

// ---------------------------------------------------------------------------------------------------------------------

static void PoseWithCovDataToMsg(const PoseWithCovData& data, geometry_msgs::msg::PoseWithCovariance& msg) {
    msg.pose.position = tf2::toMsg(data.position);
    msg.pose.orientation = tf2::toMsg(data.orientation);
    Eigen::Map<Eigen::Matrix<double, 6, 6>> cov_map = Eigen::Map<Eigen::Matrix<double, 6, 6>>(msg.covariance.data());
    cov_map = data.cov;
}

static void TwistWithCovDataToMsg(const TwistWithCovData& data, geometry_msgs::msg::TwistWithCovariance& msg) {
    tf2::toMsg(data.linear, msg.twist.linear);
    tf2::toMsg(data.angular, msg.twist.angular);
    Eigen::Map<Eigen::Matrix<double, 6, 6>> cov_map = Eigen::Map<Eigen::Matrix<double, 6, 6>>(msg.covariance.data());
    cov_map = data.cov;
}

void TfDataToTransformStamped(const TfData& data, geometry_msgs::msg::TransformStamped& msg) {
    msg.header.stamp = ros2::utils::ConvTime(data.stamp);
    msg.header.frame_id = data.frame_id;
    msg.child_frame_id = data.child_frame_id;
    msg.transform.rotation = tf2::toMsg(data.rotation);
    tf2::toMsg(data.translation, msg.transform.translation);
}

void OdometryDataToTransformStamped(const OdometryData& data, geometry_msgs::msg::TransformStamped& msg) {
    msg.header.stamp = ros2::utils::ConvTime(data.stamp);
    msg.header.frame_id = data.frame_id;
    msg.child_frame_id = data.child_frame_id;
    msg.transform.rotation = tf2::toMsg(data.pose.orientation);
    tf2::toMsg(data.pose.position, msg.transform.translation);
}

// ---------------------------------------------------------------------------------------------------------------------

template <typename SomeFpaOdoPayload, typename SomeOdoMsg>
static void FpaOdomToRos(const SomeFpaOdoPayload& payload, SomeOdoMsg& msg) {
    msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));

    msg.fusion_status = FpaFusionStatusLegacyToMsg(msg, payload.fusion_status);
    msg.imu_bias_status = FpaImuStatusLegacyToMsg(msg, payload.imu_bias_status);
    msg.gnss1_status = FpaGnssFixToMsg(msg, payload.gnss1_fix);
    msg.gnss2_status = FpaGnssFixToMsg(msg, payload.gnss2_fix);
    msg.wheelspeed_status = FpaWsStatusLegacyToMsg(msg, payload.wheelspeed_status);

    FpaFloat3ToVector3(payload.acc, msg.acceleration);

    PoseWithCovData pose;
    pose.SetFromFpaOdomPayload(payload);
    PoseWithCovDataToMsg(pose, msg.pose);

    TwistWithCovData velocity;
    velocity.SetFromFpaOdomPayload(payload);
    TwistWithCovDataToMsg(velocity, msg.velocity);
}

static void FpaOdometryToMsg(const fpa::FpaOdometryPayload& payload, fpmsgs::FpaOdometry& msg) {
    msg.header.frame_id = ODOMETRY_FRAME_ID;
    msg.pose_frame = ODOMETRY_CHILD_FRAME_ID;
    msg.kin_frame = ODOMETRY_CHILD_FRAME_ID;
    FpaOdomToRos(payload, msg);
    msg.version = payload.version;
}

void PublishFpaOdometry(const fpa::FpaOdometryPayload& payload,
                        rclcpp::Publisher<fpmsgs::FpaOdometry>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaOdometry msg;
        FpaOdometryToMsg(payload, msg);
        pub->publish(msg);
    }
}

static void FpaOdomenuToMsg(const fpa::FpaOdomenuPayload& payload, fpmsgs::FpaOdomenu& msg) {
    msg.header.frame_id = ODOMENU_FRAME_ID;
    msg.pose_frame = ODOMENU_CHILD_FRAME_ID;
    msg.kin_frame = ODOMENU_CHILD_FRAME_ID;
    FpaOdomToRos(payload, msg);
}

void PublishFpaOdomenu(const fpa::FpaOdomenuPayload& payload, rclcpp::Publisher<fpmsgs::FpaOdomenu>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaOdomenu msg;
        FpaOdomenuToMsg(payload, msg);
        FpaOdomToRos(payload, msg);
        pub->publish(msg);
    }
}

static void FpaOdomshToMsg(const fpa::FpaOdomshPayload& payload, fpmsgs::FpaOdomsh& msg) {
    msg.header.frame_id = ODOMSH_FRAME_ID;
    msg.pose_frame = ODOMSH_CHILD_FRAME_ID;
    msg.kin_frame = ODOMSH_CHILD_FRAME_ID;
    FpaOdomToRos(payload, msg);
}

void PublishFpaOdomsh(const fpa::FpaOdomshPayload& payload, rclcpp::Publisher<fpmsgs::FpaOdomsh>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaOdomsh msg;
        FpaOdomshToMsg(payload, msg);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFpaOdometryDataImu(const fpa::FpaOdomenuPayload& payload, bool nav2_mode_,
                               rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        // Only publish if data is valid
        if (!payload.orientation.valid || !payload.acc.valid || !payload.rot.valid) {
            return;
        }
        if (payload.gnss1_fix != fpa::FpaGnssFix::RTK_FIXED || payload.gnss2_fix != fpa::FpaGnssFix::RTK_FIXED) {
            return;
        }
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
        msg.header.frame_id = nav2_mode_ ? "vrtk_link" : ODOMETRY_FRAME_ID;

        // Orientation quaternion
        const Eigen::Quaterniond quat = {payload.orientation.values[0], payload.orientation.values[1],
                                         payload.orientation.values[2], payload.orientation.values[3]};
        msg.orientation = tf2::toMsg(quat);

        // Angular velocity and acceleration in body frame
        FpaFloat3ToVector3(payload.rot, msg.angular_velocity);
        FpaFloat3ToVector3(payload.acc, msg.linear_acceleration);

        // Orientation covariance
        if (payload.orientation_cov.valid) {
            Eigen::Matrix3d ori_cov =
                BuildCovMat3D(payload.orientation_cov.values[0], payload.orientation_cov.values[1],
                              payload.orientation_cov.values[2], payload.orientation_cov.values[3],
                              payload.orientation_cov.values[4], payload.orientation_cov.values[5]);
            for (int i = 0; i < 9; ++i) {
                msg.orientation_covariance[i] = ori_cov(i / 3, i % 3);
            }
        } else {
            std::fill(msg.orientation_covariance.begin(), msg.orientation_covariance.end(), -1.0);
        }

        // Linear acceleration covariance
        if (payload.vel_cov.valid) {
            Eigen::Matrix3d vel_cov =
                BuildCovMat3D(payload.vel_cov.values[0], payload.vel_cov.values[1], payload.vel_cov.values[2],
                              payload.vel_cov.values[3], payload.vel_cov.values[4], payload.vel_cov.values[5]);
            const double accel_scale = 0.1;
            Eigen::Matrix3d acc_cov = vel_cov * accel_scale;
            for (int i = 0; i < 9; ++i) {
                msg.linear_acceleration_covariance[i] = acc_cov(i / 3, i % 3);
            }
        } else {
            std::fill(msg.linear_acceleration_covariance.begin(), msg.linear_acceleration_covariance.end(), 0.0);
            msg.linear_acceleration_covariance[0] = 0.01;
            msg.linear_acceleration_covariance[4] = 0.01;
            msg.linear_acceleration_covariance[8] = 0.01;
        }

        // Angular velocity covariance
        if (payload.orientation_cov.valid) {
            Eigen::Matrix3d ori_cov =
                BuildCovMat3D(payload.orientation_cov.values[0], payload.orientation_cov.values[1],
                              payload.orientation_cov.values[2], payload.orientation_cov.values[3],
                              payload.orientation_cov.values[4], payload.orientation_cov.values[5]);
            const double ang_vel_scale = 0.01;
            Eigen::Matrix3d ang_vel_cov = ori_cov * ang_vel_scale;
            for (int i = 0; i < 9; ++i) {
                msg.angular_velocity_covariance[i] = ang_vel_cov(i / 3, i % 3);
            }
        } else {
            std::fill(msg.angular_velocity_covariance.begin(), msg.angular_velocity_covariance.end(), 0.0);
            msg.angular_velocity_covariance[0] = 0.001;
            msg.angular_velocity_covariance[4] = 0.001;
            msg.angular_velocity_covariance[8] = 0.001;
        }

        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFpaOdometryDataNavSatFix(const fpa::FpaOdometryPayload& payload, bool nav2_mode_,
                                     rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
        if (nav2_mode_) {
            msg.header.frame_id = "vrtk_link";
        } else {
            msg.header.frame_id = ODOMETRY_CHILD_FRAME_ID;
        }

        // Populate LLH position
        PoseWithCovData pose;
        pose.SetFromFpaOdomPayload(payload);
        Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map =
            Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.position_covariance.data());

        const Eigen::Vector3d llh_pos = trafo::TfWgs84LlhEcef(pose.position);
        msg.latitude = math::RadToDeg(llh_pos(0));
        msg.longitude = math::RadToDeg(llh_pos(1));
        msg.altitude = llh_pos(2);

        // Populate LLH covariance
        const Eigen::Matrix3d p_cov_e = pose.cov.topLeftCorner(3, 3);
        const Eigen::Matrix3d C_l_e = trafo::RotEnuEcef(pose.position);
        const Eigen::Matrix3d p_cov_l = C_l_e * p_cov_e * C_l_e.transpose();
        cov_map = p_cov_l;
        msg.position_covariance_type = msg.COVARIANCE_TYPE_KNOWN;

        // Populate LLH status
        if (nav2_mode_) {
            msg.status.status = 2;
            msg.status.service = 15;
        } else {
            const fpa::FpaGnssFix fix = (payload.gnss1_fix > payload.gnss2_fix ? payload.gnss1_fix : payload.gnss2_fix);
            msg.status.status = FpaGnssFixToNavSatStatusStatus(msg.status, fix);
            if (msg.status.status != msg.status.STATUS_NO_FIX) {
                msg.status.service = (msg.status.SERVICE_GPS | msg.status.SERVICE_GLONASS | msg.status.SERVICE_COMPASS |
                                      msg.status.SERVICE_GALILEO);
            }
        }
        if (payload.gnss1_fix != fpa::FpaGnssFix::RTK_FIXED || payload.gnss2_fix != fpa::FpaGnssFix::RTK_FIXED) {
            return;
        }
        // Publish message
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFpaOdomenuVector3Stamped(const fpa::FpaOdomenuPayload& payload,
                                     rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        geometry_msgs::msg::Vector3Stamped msg;

        msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
        msg.header.frame_id = ENU_FRAME_ID;

        const Eigen::Quaterniond quat = {payload.orientation.values[0], payload.orientation.values[1],
                                         payload.orientation.values[2], payload.orientation.values[3]};

        // Euler angle wrt. ENU frame in the order of yaw pitch roll
        const Eigen::Vector3d enu_euler = trafo::RotToEul(quat.toRotationMatrix());
        // const Eigen::Vector3d enu_euler = trafo::QuatToEul(quat); // FIXME couldn't we just use this?
        msg.vector.set__x(enu_euler.x());
        msg.vector.set__y(enu_euler.y());
        msg.vector.set__z(enu_euler.z());

        // Publish message
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

static void FpaOdomstatusToMsg(const fpa::FpaOdomstatusPayload& payload, fpmsgs::FpaOdomstatus& msg) {
    // clang-format off
    msg.header.stamp    = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
    msg.init_status     = FpaInitStatusToMsg(msg, payload.init_status);
    msg.fusion_imu      = FpaMeasStatusToMsg(msg, payload.fusion_imu);
    msg.fusion_gnss1    = FpaMeasStatusToMsg(msg, payload.fusion_gnss1);
    msg.fusion_gnss2    = FpaMeasStatusToMsg(msg, payload.fusion_gnss2);
    msg.fusion_corr     = FpaMeasStatusToMsg(msg, payload.fusion_corr);
    msg.fusion_cam1     = FpaMeasStatusToMsg(msg, payload.fusion_cam1);
    msg.fusion_ws       = FpaMeasStatusToMsg(msg, payload.fusion_ws);
    msg.fusion_markers  = FpaMeasStatusToMsg(msg, payload.fusion_markers);
    msg.imu_status      = FpaImuStatusToMsg(msg, payload.imu_status);
    msg.imu_noise       = FpaImuNoiseToMsg(msg, payload.imu_noise);
    msg.imu_conv        = FpaImuConvToMsg(msg, payload.imu_conv);
    msg.gnss1_status    = FpaGnssStatusToMsg(msg, payload.gnss1_status);
    msg.gnss2_status    = FpaGnssStatusToMsg(msg, payload.gnss2_status);
    msg.baseline_status = FpaBaselineStatusToMsg(msg, payload.baseline_status);
    msg.corr_status     = FpaCorrStatusToMsg(msg, payload.corr_status);
    msg.cam1_status     = FpaCamStatusToMsg(msg, payload.cam1_status);
    msg.ws_status       = FpaWsStatusToMsg(msg, payload.ws_status);
    msg.ws_conv         = FpaWsConvToMsg(msg, payload.ws_conv);
    msg.markers_status  = FpaMarkersStatusToMsg(msg, payload.markers_status);
    msg.markers_conv    = FpaMarkersConvToMsg(msg, payload.markers_conv);
    // clang-format on
}

void PublishFpaOdomstatus(const fpa::FpaOdomstatusPayload& payload,
                          rclcpp::Publisher<fpmsgs::FpaOdomstatus>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaOdomstatus msg;
        FpaOdomstatusToMsg(payload, msg);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFpaLlh(const fpa::FpaLlhPayload& payload, rclcpp::Publisher<fpmsgs::FpaLlh>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaLlh msg;
        msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
        msg.header.frame_id = ODOMETRY_CHILD_FRAME_ID;
        FpaFloat3ToVector3(payload.llh, msg.position);
        if (payload.cov_enu.valid) {
            Eigen::Map<Eigen::Matrix3d> cov_map = Eigen::Map<Eigen::Matrix3d>(msg.covariance.data());
            cov_map = BuildCovMat3D(payload.cov_enu.values[0], payload.cov_enu.values[1], payload.cov_enu.values[2],
                                    payload.cov_enu.values[3], payload.cov_enu.values[4], payload.cov_enu.values[5]);
        }
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFpaEoe(const fpa::FpaEoePayload& payload, rclcpp::Publisher<fpmsgs::FpaEoe>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaEoe msg;
        msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
        msg.epoch = FpaEpochToMsg(msg, payload.epoch);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

static void FpaImubiasToMsg(const fpa::FpaImubiasPayload& payload, fpmsgs::FpaImubias& msg) {
    msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
    msg.header.frame_id = IMU_FRAME_ID;
    msg.fusion_imu = FpaMeasStatusToMsg(msg, payload.fusion_imu);
    msg.imu_status = FpaImuStatusToMsg(msg, payload.imu_status);
    msg.imu_noise = FpaImuNoiseToMsg(msg, payload.imu_noise);
    msg.imu_conv = FpaImuConvToMsg(msg, payload.imu_conv);
    FpaFloat3ToVector3(payload.bias_acc, msg.bias_acc);
    FpaFloat3ToVector3(payload.bias_gyr, msg.bias_gyr);
    FpaFloat3ToVector3(payload.bias_cov_acc, msg.bias_cov_acc);
    FpaFloat3ToVector3(payload.bias_cov_gyr, msg.bias_cov_gyr);
}

void PublishFpaImubias(const fpa::FpaImubiasPayload& payload, rclcpp::Publisher<fpmsgs::FpaImubias>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaImubias msg;
        FpaImubiasToMsg(payload, msg);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFpaGnssant(const fpa::FpaGnssantPayload& payload, rclcpp::Publisher<fpmsgs::FpaGnssant>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaGnssant msg;
        msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
        msg.gnss1_state = FpaAntStateToMsg(msg, payload.gnss1_state);
        msg.gnss1_power = FpaAntPowerToMsg(msg, payload.gnss1_power);
        msg.gnss1_age = (payload.gnss1_age.valid ? payload.gnss1_age.value : -1);
        msg.gnss2_state = FpaAntStateToMsg(msg, payload.gnss2_state);
        msg.gnss2_power = FpaAntPowerToMsg(msg, payload.gnss2_power);
        msg.gnss2_age = (payload.gnss2_age.valid ? payload.gnss2_age.value : -1);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFpaGnsscorr(const fpa::FpaGnsscorrPayload& payload,
                        rclcpp::Publisher<fpmsgs::FpaGnsscorr>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaGnsscorr msg;
        msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
        msg.gnss1_fix = FpaGnssFixToMsg(msg, payload.gnss1_fix);
        msg.gnss1_nsig_l1 = (payload.gnss1_nsig_l1.valid ? payload.gnss1_nsig_l1.value : -1);
        msg.gnss1_nsig_l2 = (payload.gnss1_nsig_l2.valid ? payload.gnss1_nsig_l2.value : -1);
        msg.gnss2_fix = FpaGnssFixToMsg(msg, payload.gnss2_fix);
        msg.gnss2_nsig_l1 = (payload.gnss2_nsig_l1.valid ? payload.gnss2_nsig_l1.value : -1);
        msg.gnss2_nsig_l2 = (payload.gnss2_nsig_l2.valid ? payload.gnss2_nsig_l2.value : -1);
        msg.corr_latency = (payload.corr_latency.valid ? payload.corr_latency.value : 0.0f);
        msg.corr_update_rate = (payload.corr_update_rate.valid ? payload.corr_update_rate.value : 0.0f);
        msg.corr_data_rate = (payload.corr_data_rate.valid ? payload.corr_data_rate.value : 0.0f);
        msg.corr_msg_rate = (payload.corr_msg_rate.valid ? payload.corr_msg_rate.value : 0.0f);
        msg.sta_id = (payload.sta_id.valid ? payload.sta_id.value : -1);
        FpaFloat3ToVector3(payload.sta_llh, msg.sta_llh);
        msg.sta_dist = (payload.sta_dist.valid ? payload.sta_dist.value : -1);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFpaTp(const fpa::FpaTpPayload& payload, rclcpp::Publisher<fpmsgs::FpaTp>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaTp msg;
        msg.tp_name = payload.tp_name;
        msg.timebase = FpaTimebaseToMsg(msg, payload.timebase);
        msg.timeref = FpaTimerefToMsg(msg, payload.timeref);
        msg.tp_week = payload.tp_week.value;
        msg.tp_tow_sec = payload.tp_tow_sec.value;
        msg.tp_tow_psec = payload.tp_tow_psec.value;
        msg.gps_leaps = payload.gps_leaps.value;
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFpaText(const fpa::FpaTextPayload& payload, rclcpp::Publisher<fpmsgs::FpaText>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FpaText msg;
        msg.level = FpaTextLevelToMsg(msg, payload.level);
        msg.text = payload.text;
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

template <typename SomeFpaImuPayload>
static void FpaImuPayloadToRos(const SomeFpaImuPayload& payload, sensor_msgs::msg::Imu& msg) {
    msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(payload.gps_time));
    msg.header.frame_id = IMU_FRAME_ID;
    if (payload.acc.valid) {
        msg.linear_acceleration.x = payload.acc.values[0];
        msg.linear_acceleration.y = payload.acc.values[1];
        msg.linear_acceleration.z = payload.acc.values[2];
    }
    if (payload.rot.valid) {
        msg.angular_velocity.x = payload.rot.values[0];
        msg.angular_velocity.y = payload.rot.values[1];
        msg.angular_velocity.z = payload.rot.values[2];
    }
}

void PublishFpaRawimu(const fpa::FpaRawimuPayload& payload, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        sensor_msgs::msg::Imu msg;
        FpaImuPayloadToRos(payload, msg);
        pub->publish(msg);
    }
}

void PublishFpaCorrimu(const fpa::FpaCorrimuPayload& payload,
                       rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        sensor_msgs::msg::Imu msg;
        FpaImuPayloadToRos(payload, msg);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool PublishNovbBestgnsspos(const novb::NovbHeader* header, const novb::NovbBestgnsspos* payload,
                            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& pub1,
                            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& pub2) {
    if ((header == NULL) || (payload == NULL) || !header->IsLongHeader() ||
        !((header->Source() == novb::NovbMsgTypeSource::PRIMARY) ||
          (header->Source() == novb::NovbMsgTypeSource::SECONDARY))) {
        return false;
    }
    auto pub = (header->Source() == novb::NovbMsgTypeSource::PRIMARY ? pub1 : pub2);
    if (pub->get_subscription_count() > 0) {
        sensor_msgs::msg::NavSatFix msg;

        time::Time stamp;
        if (stamp.SetWnoTow({header->long_header.gps_week, (double)header->long_header.gps_milliseconds * 1e-3,
                             time::WnoTow::Sys::GPS})) {
            msg.header.stamp = ros2::utils::ConvTime(stamp);
        }

        msg.header.frame_id = (header->Source() == novb::NovbMsgTypeSource::PRIMARY ? GNSS1_FRAME_ID : GNSS2_FRAME_ID);
        msg.status.status =
            NovbPosOrVelTypeToNavSatStatusStatus(msg.status, static_cast<novb::NovbPosOrVelType>(payload->pos_type));
        if (msg.status.status != msg.status.STATUS_NO_FIX) {
            msg.status.service = (msg.status.SERVICE_GPS | msg.status.SERVICE_GLONASS | msg.status.SERVICE_COMPASS |
                                  msg.status.SERVICE_GALILEO);
        }

        msg.latitude = payload->lat;
        msg.longitude = payload->lon;
        msg.altitude = payload->hgt;

        msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
        Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map =
            Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.position_covariance.data());

        Eigen::Array3d cov_diag(payload->lat_stdev, payload->lon_stdev, payload->hgt_stdev);
        cov_map = (cov_diag * cov_diag).matrix().asDiagonal();

        pub->publish(msg);
    }
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

static void NovbInspvaxToMsg(const novb::NovbHeader* header, const novb::NovbInspvax* payload,
                             fpmsgs::NovbInspvax& msg) {
    if ((header != NULL) && (payload != NULL) && header->IsLongHeader()) {
        time::Time stamp;
        if (stamp.SetWnoTow({header->long_header.gps_week, (double)header->long_header.gps_milliseconds * 1e-3,
                             time::WnoTow::Sys::GPS})) {
            msg.header.stamp = ros2::utils::ConvTime(stamp);
        }

        msg.ins_status = payload->ins_status;
        msg.pos_type = payload->pos_type;
        msg.latitude = payload->latitude;
        msg.longitude = payload->longitude;
        msg.height = payload->height;
        msg.undulation = payload->undulation;
        msg.north_velocity = payload->north_velocity;
        msg.east_velocity = payload->east_velocity;
        msg.up_velocity = payload->up_velocity;
        msg.roll = payload->roll;
        msg.pitch = payload->pitch;
        msg.azimuth = payload->azimuth;
        msg.latitude_stdev = payload->latitude_stdev;
        msg.longitude_stdev = payload->longitude_stdev;
        msg.height_stdev = payload->height_stdev;
        msg.north_velocity_stdev = payload->north_velocity_stdev;
        msg.east_velocity_stdev = payload->east_velocity_stdev;
        msg.up_velocity_stdev = payload->up_velocity_stdev;
        msg.roll_stdev = payload->roll_stdev;
        msg.pitch_stdev = payload->pitch_stdev;
        msg.azimuth_stdev = payload->azimuth_stdev;
        msg.extended_status = payload->extended_status;
        msg.time_since_update = payload->time_since_update;
    }
}

bool PublishNovbInspvax(const novb::NovbHeader* header, const novb::NovbInspvax* payload,
                        rclcpp::Publisher<fpmsgs::NovbInspvax>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NovbInspvax msg;
        NovbInspvaxToMsg(header, payload, msg);
        pub->publish(msg);
    }
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaGga(const fpsdk::common::parser::nmea::NmeaGgaPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGga>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaGga msg;
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        if (payload.time.valid) {
            msg.time_valid = true;
            msg.time_h = payload.time.hours;
            msg.time_m = payload.time.mins;
            msg.time_s = payload.time.secs;
        }
        msg.latitude = (payload.llh.latlon_valid ? payload.llh.lat : NAN);
        msg.longitude = (payload.llh.latlon_valid ? payload.llh.lon : NAN);
        msg.height = (payload.llh.height_valid ? payload.llh.height : NAN);
        msg.quality = NmeaQualityGgaToMsg(msg, payload.quality);
        msg.num_sv = (payload.num_sv.valid ? payload.num_sv.value : -1);
        msg.hdop = (payload.hdop.valid ? payload.hdop.value : NAN);
        msg.diff_age = (payload.diff_age.valid ? payload.diff_age.value : NAN);
        msg.diff_sta = payload.diff_sta.value;
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaGll(const fpsdk::common::parser::nmea::NmeaGllPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGll>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaGll msg;
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        if (payload.time.valid) {
            msg.time_valid = true;
            msg.time_h = payload.time.hours;
            msg.time_m = payload.time.mins;
            msg.time_s = payload.time.secs;
        }
        msg.latitude = (payload.ll.latlon_valid ? payload.ll.lat : NAN);
        msg.longitude = (payload.ll.latlon_valid ? payload.ll.lon : NAN);
        msg.status = NmeaStatusGllRmcToMsg(msg, payload.status);
        msg.mode = NmeaModeGllVtgToMsg(msg, payload.mode);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaGsa(const fpsdk::common::parser::nmea::NmeaGsaPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGsa>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaGsa msg;
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        msg.system = NmeaSystemIdToMsg(msg, payload.system);
        msg.opmode = NmeaOpModeGsaToMsg(msg, payload.opmode);
        msg.navmode = NmeaNavModeGsaToMsg(msg, payload.navmode);
        for (auto& sat : payload.sats) {
            if (sat.valid) {
                msg.sats.push_back(sat.svid);
            }
        }
        msg.pdop = (payload.pdop.valid ? payload.pdop.value : NAN);
        msg.hdop = (payload.hdop.valid ? payload.hdop.value : NAN);
        msg.vdop = (payload.vdop.valid ? payload.vdop.value : NAN);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaGst(const fpsdk::common::parser::nmea::NmeaGstPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGst>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaGst msg;
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        if (payload.time.valid) {
            msg.time_valid = true;
            msg.time_h = payload.time.hours;
            msg.time_m = payload.time.mins;
            msg.time_s = payload.time.secs;
        }
        msg.rms_range = (payload.rms_range.valid ? payload.rms_range.value : NAN);
        msg.std_major = (payload.std_major.valid ? payload.std_major.value : NAN);
        msg.std_minor = (payload.std_minor.valid ? payload.std_minor.value : NAN);
        msg.angle_major = (payload.angle_major.valid ? payload.angle_major.value : NAN);
        msg.std_lat = (payload.std_lat.valid ? payload.std_lat.value : NAN);
        msg.std_lon = (payload.std_lon.valid ? payload.std_lon.value : NAN);
        msg.std_alt = (payload.std_alt.valid ? payload.std_alt.value : NAN);
        pub->publish(msg);
    }
}
// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaGsv(const fpsdk::common::parser::nmea::NmeaGsvPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaGsv>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaGsv msg;
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        msg.system = NmeaSystemIdToMsg(msg, payload.system);
        msg.signal = NmeaSignalIdToMsg(msg, payload.signal);
        msg.num_msgs = payload.num_msgs.value;
        msg.msg_num = payload.msg_num.value;
        for (auto& azel : payload.azels) {
            if (azel.valid) {
                msg.azel_sat.push_back(azel.svid);
                msg.el.push_back(azel.el);
                msg.az.push_back(azel.az);
            }
        }
        for (auto& cno : payload.cnos) {
            if (cno.valid) {
                msg.cno_sat.push_back(cno.svid);
                msg.cno.push_back(cno.cno);
            }
        }
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaHdt(const fpsdk::common::parser::nmea::NmeaHdtPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaHdt>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaHdt msg;
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        msg.heading = (payload.heading.valid ? payload.heading.value : NAN);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaRmc(const fpsdk::common::parser::nmea::NmeaRmcPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaRmc>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaRmc msg;
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        if (payload.date.valid) {
            msg.date_valid = true;
            msg.date_y = payload.date.years;
            msg.date_m = payload.date.months;
            msg.date_d = payload.date.days;
        }
        if (payload.time.valid) {
            msg.time_valid = true;
            msg.time_h = payload.time.hours;
            msg.time_m = payload.time.mins;
            msg.time_s = payload.time.secs;
        }
        msg.status = NmeaStatusGllRmcToMsg(msg, payload.status);
        msg.mode = NmeaModeRmcGnsToMsg(msg, payload.mode);
        msg.navstatus = NmeaNavStatusRmcToMsg(msg, payload.navstatus);
        msg.latitude = (payload.llh.latlon_valid ? payload.llh.lat : NAN);
        msg.longitude = (payload.llh.latlon_valid ? payload.llh.lon : NAN);
        msg.speed = (payload.speed.valid ? payload.speed.value : NAN);
        msg.course = (payload.course.valid ? payload.course.value : NAN);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaVtg(const fpsdk::common::parser::nmea::NmeaVtgPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaVtg>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaVtg msg;
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        msg.cogt = (payload.cogt.valid ? payload.cogt.value : NAN);
        msg.cogm = (payload.cogm.valid ? payload.cogm.value : NAN);
        msg.sogn = (payload.sogn.valid ? payload.sogn.value : NAN);
        msg.sogk = (payload.sogk.valid ? payload.sogk.value : NAN);
        msg.mode = NmeaModeGllVtgToMsg(msg, payload.mode);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaZda(const fpsdk::common::parser::nmea::NmeaZdaPayload& payload,
                    rclcpp::Publisher<fpmsgs::NmeaZda>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaZda msg;
        msg.talker = NmeaTalkerIdToMsg(msg, payload.talker);
        if (payload.date.valid) {
            msg.date_valid = true;
            msg.date_y = payload.date.years;
            msg.date_m = payload.date.months;
            msg.date_d = payload.date.days;
        }
        if (payload.time.valid) {
            msg.time_valid = true;
            msg.time_h = payload.time.hours;
            msg.time_m = payload.time.mins;
            msg.time_s = payload.time.secs;
        }
        msg.local_hr = payload.local_hr.value;
        msg.local_min = payload.local_min.value;
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishParserMsg(const fpsdk::common::parser::ParserMsg& msg,
                      rclcpp::Publisher<fpmsgs::ParserMsg>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::ParserMsg ros_msg;
        ros_msg.protocol = ParserProtocolToMsg(ros_msg, msg.proto_);
        ros_msg.data = msg.data_;
        ros_msg.name = msg.name_;
        ros_msg.seq = msg.seq_;
        msg.MakeInfo();
        ros_msg.info = msg.info_;
        pub->publish(ros_msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishNmeaEpochData(const NmeaEpochData& data, rclcpp::Publisher<fpmsgs::NmeaEpoch>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::NmeaEpoch msg;
        msg.header.stamp = ros2::utils::ConvTime(data.stamp_);
        msg.header.frame_id = data.frame_id_;
        if (data.date_.valid) {
            msg.date_valid = true;
            msg.date_y = data.date_.years;
            msg.date_m = data.date_.months;
            msg.date_d = data.date_.days;
        }
        if (data.time_.valid) {
            msg.time_valid = true;
            msg.time_h = data.time_.hours;
            msg.time_m = data.time_.mins;
            msg.time_s = data.time_.secs;
        }
        msg.status = NmeaStatusGllRmcToMsg(msg, data.status_);
        msg.navstatus = NmeaNavStatusRmcToMsg(msg, data.navstatus_);
        msg.mode1 = NmeaModeRmcGnsToMsg(msg, data.mode1_);
        msg.mode2 = NmeaModeGllVtgToMsg(msg, data.mode2_);
        msg.quality = NmeaQualityGgaToMsg(msg, data.quality_);
        msg.opmode = NmeaOpModeGsaToMsg(msg, data.opmode_);
        msg.navmode = NmeaNavModeGsaToMsg(msg, data.navmode_);
        msg.latitude = (data.llh_.latlon_valid ? data.llh_.lat : NAN);
        msg.longitude = (data.llh_.latlon_valid ? data.llh_.lon : NAN);
        msg.height = (data.llh_.height_valid ? data.llh_.height : NAN);
        msg.num_sv = (data.num_sv_.valid ? data.num_sv_.value : -1);
        msg.rms_range = (data.rms_range_.valid ? data.rms_range_.value : NAN);
        msg.std_major = (data.std_major_.valid ? data.std_major_.value : NAN);
        msg.std_minor = (data.std_minor_.valid ? data.std_minor_.value : NAN);
        msg.angle_major = (data.angle_major_.valid ? data.angle_major_.value : NAN);
        msg.std_lat = (data.std_lat_.valid ? data.std_lat_.value : NAN);
        msg.std_lon = (data.std_lon_.valid ? data.std_lon_.value : NAN);
        msg.std_alt = (data.std_alt_.valid ? data.std_alt_.value : NAN);
        msg.pdop = (data.pdop_.valid ? data.pdop_.value : NAN);
        msg.hdop = (data.hdop_.valid ? data.hdop_.value : NAN);
        msg.vdop = (data.vdop_.valid ? data.vdop_.value : NAN);
        msg.heading = (data.heading_.valid ? data.heading_.value : NAN);
        msg.speed = (data.speed_.valid ? data.speed_.value : NAN);
        msg.course = (data.course_.valid ? data.course_.value : NAN);
        msg.cogt = (data.cogt_.valid ? data.cogt_.value : NAN);
        msg.cogm = (data.cogm_.valid ? data.cogm_.value : NAN);
        msg.sogn = (data.sogn_.valid ? data.sogn_.value : NAN);
        msg.sogk = (data.sogk_.valid ? data.sogk_.value : NAN);
        msg.diff_age = (data.diff_age_.valid ? data.diff_age_.value : NAN);
        msg.diff_sta = data.diff_sta_.value;
        for (auto& sat : data.gsa_gsv_.sats_) {
            fpmsgs::NmeaSatellite sat_msg;
            sat_msg.system = NmeaSystemIdToMsg(sat_msg, sat.system_);
            sat_msg.svid = sat.svid_;
            sat_msg.az = sat.az_;
            sat_msg.el = sat.el_;
            msg.sats.push_back(sat_msg);
        }
        for (auto& sig : data.gsa_gsv_.sigs_) {
            fpmsgs::NmeaSignal sig_msg;
            sig_msg.system = NmeaSystemIdToMsg(sig_msg, sig.system_);
            sig_msg.svid = sig.svid_;
            sig_msg.signal = NmeaSignalIdToMsg(msg, sig.signal_);
            sig_msg.cno = sig.cno_;
            sig_msg.used = sig.used_;
            msg.sigs.push_back(sig_msg);
        }

        Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map = Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.cov_enu.data());
        cov_map = data.cov_enu_;

        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishOdometryData(const OdometryData& data, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = ros2::utils::ConvTime(data.stamp);
        msg.header.frame_id = data.frame_id;
        msg.child_frame_id = data.child_frame_id;
        PoseWithCovDataToMsg(data.pose, msg.pose);
        TwistWithCovDataToMsg(data.twist, msg.twist);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishJumpWarning(const JumpDetector& jump_detector, rclcpp::Publisher<fpmsgs::CovWarn>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::CovWarn msg;
        msg.header.stamp = ros2::utils::ConvTime(jump_detector.curr_stamp_);
        tf2::toMsg(jump_detector.pos_diff_, msg.jump);
        msg.covariance.x = jump_detector.prev_cov_(0, 0);
        msg.covariance.y = jump_detector.prev_cov_(1, 1);
        msg.covariance.z = jump_detector.prev_cov_(2, 2);
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishDatum(const geometry_msgs::msg::Vector3& payload, const builtin_interfaces::msg::Time& stamp,
                  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "vrtk_link";

        // Populate LLH position
        const Eigen::Vector3d position = {payload.x, payload.y, payload.z};
        const Eigen::Vector3d llh_pos = trafo::TfWgs84LlhEcef(position);
        msg.latitude = math::RadToDeg(llh_pos(0));
        msg.longitude = math::RadToDeg(llh_pos(1));
        msg.altitude = llh_pos(2);

        // Populate status
        msg.status.status = 2;
        msg.status.service = 15;
        msg.position_covariance_type = 3;

        // Publish message
        pub->publish(msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PublishFusionEpochData(const FusionEpochData& data, rclcpp::Publisher<fpmsgs::FusionEpoch>::SharedPtr& pub) {
    if (pub->get_subscription_count() > 0) {
        fpmsgs::FusionEpoch msg;
        msg.header.stamp = ros2::utils::ConvTime(FpaGpsTimeToTime(data.fpa_eoe_.gps_time));
        if (data.fpa_odometry_avail_) {
            msg.fpa_odometry_avail = true;
            FpaOdometryToMsg(data.fpa_odometry_, msg.fpa_odometry);
        }
        if (data.fpa_odomsh_avail_) {
            msg.fpa_odomsh_avail = true;
            FpaOdomshToMsg(data.fpa_odomsh_, msg.fpa_odomsh);
        }
        if (data.fpa_odomenu_avail_) {
            msg.fpa_odomenu_avail = true;
            FpaOdomenuToMsg(data.fpa_odomenu_, msg.fpa_odomenu);
        }
        if (data.fpa_odomstatus_avail_) {
            msg.fpa_odomstatus_avail = true;
            FpaOdomstatusToMsg(data.fpa_odomstatus_, msg.fpa_odomstatus);
        }
        if (data.novb_inspvax_avail_) {
            msg.novb_inspvax_avail = true;
            NovbInspvaxToMsg(&data.novb_inspvax_header_, &data.novb_inspvax_payload_, msg.novb_inspvax);
        }
        if (data.fpa_imubias_avail_) {
            msg.fpa_imubias_avail = true;
            FpaImubiasToMsg(data.fpa_imubias_, msg.fpa_imubias);
        }
        pub->publish(msg);
    }
}
/* ****************************************************************************************************************** */
}  // namespace fixposition
