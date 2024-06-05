/**
 *  @file
 *  @brief Convert Data classes to ROS1 msgs
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
#include <fixposition_driver_ros1/data_to_ros1.hpp>

namespace fixposition {
void ImuDataToMsg(const ImuData& data, sensor_msgs::Imu& msg) {
    if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    }
    
    msg.header.frame_id = data.frame_id;
    tf::vectorEigenToMsg(data.linear_acceleration, msg.linear_acceleration);
    tf::vectorEigenToMsg(data.angular_velocity, msg.angular_velocity);
}

void NavSatStatusDataToMsg(const NavSatStatusData& data, sensor_msgs::NavSatStatus& msg) {
    msg.status = data.status;
    msg.service = data.service;
}

void NavSatFixDataToMsg(const NavSatFixData& data, sensor_msgs::NavSatFix& msg) {
    if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    }

    msg.header.frame_id = data.frame_id;
    NavSatStatusDataToMsg(data.status, msg.status);
    msg.latitude = data.latitude;
    msg.longitude = data.longitude;
    msg.altitude = data.altitude;

    Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map =
        Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.position_covariance.data());
    cov_map = data.cov;

    msg.position_covariance_type = data.position_covariance_type;
}

void PoseWithCovDataToMsg(const PoseWithCovData& data, geometry_msgs::PoseWithCovariance& msg) {
    tf::pointEigenToMsg(data.position, msg.pose.position);
    tf::quaternionEigenToMsg(data.orientation, msg.pose.orientation);

    Eigen::Map<Eigen::Matrix<double, 6, 6>> cov_map = Eigen::Map<Eigen::Matrix<double, 6, 6>>(msg.covariance.data());
    cov_map = data.cov;
}

void TwistWithCovDataToMsg(const fixposition::TwistWithCovData& data, geometry_msgs::TwistWithCovariance& msg) {
    tf::vectorEigenToMsg(data.linear, msg.twist.linear);
    tf::vectorEigenToMsg(data.angular, msg.twist.angular);

    Eigen::Map<Eigen::Matrix<double, 6, 6>> cov_map = Eigen::Map<Eigen::Matrix<double, 6, 6>>(msg.covariance.data());
    cov_map = data.cov;
}

void OdometryDataToMsg(const fixposition::OdometryData& data, nav_msgs::Odometry& msg) {
    if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    }

    msg.header.frame_id = data.frame_id;
    msg.child_frame_id = data.child_frame_id;

    PoseWithCovDataToMsg(data.pose, msg.pose);
    TwistWithCovDataToMsg(data.twist, msg.twist);
}

void OdometryDataToTf(const OdometryData& data, geometry_msgs::TransformStamped& msg) {
    msg.header.frame_id = data.frame_id;
    msg.child_frame_id = data.child_frame_id;

    if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    }

    tf::quaternionEigenToMsg(data.pose.orientation, msg.transform.rotation);
    tf::vectorEigenToMsg(data.pose.position, msg.transform.translation);
}

void OdomToNavSatFix(const fixposition::FP_ODOMETRY& data, sensor_msgs::NavSatFix& msg) {
    // Populate message header
    if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.odom.stamp));
    }
    msg.header.frame_id = data.odom.frame_id;
    
    // Populate LLH position
    const Eigen::Vector3d llh_pos = gnss_tf::TfWgs84LlhEcef(data.odom.pose.position);
    msg.latitude  = RadToDeg(llh_pos(0));
    msg.longitude = RadToDeg(llh_pos(1));
    msg.altitude  = llh_pos(2);

    // Populate LLH covariance
    const Eigen::Matrix3d p_cov_e = data.odom.pose.cov.topLeftCorner(3, 3);
    const Eigen::Matrix3d C_l_e = gnss_tf::RotEnuEcef(data.odom.pose.position);
    const Eigen::Matrix3d p_cov_l = C_l_e * p_cov_e * C_l_e.transpose();
    
    Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map =
        Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.position_covariance.data());
    cov_map = p_cov_l;
    msg.position_covariance_type = 3;

    // Populate LLH status
    int status_flag = std::max(data.gnss1_status, data.gnss2_status);

    if (status_flag < static_cast<int8_t>(GnssStatus::FIX_TYPE_S2D)) {
        msg.status.status = static_cast<int8_t>(NavSatStatusData::Status::STATUS_NO_FIX);
        msg.status.service = static_cast<uint16_t>(NavSatStatusData::Service::SERVICE_NONE);

    } else if (status_flag >= static_cast<int8_t>(GnssStatus::FIX_TYPE_S2D) || status_flag < static_cast<int8_t>(GnssStatus::FIX_TYPE_RTK_FLOAT)) {
        msg.status.status = static_cast<int8_t>(NavSatStatusData::Status::STATUS_FIX);
        msg.status.service = static_cast<uint16_t>(NavSatStatusData::Service::SERVICE_ALL);

    } else if (status_flag >= static_cast<int8_t>(GnssStatus::FIX_TYPE_RTK_FLOAT)) {
        msg.status.status = static_cast<int8_t>(NavSatStatusData::Status::STATUS_GBAS_FIX);
        msg.status.service = static_cast<uint16_t>(NavSatStatusData::Service::SERVICE_ALL);

    } else {
        msg.status.status = static_cast<int8_t>(NavSatStatusData::Status::STATUS_NO_FIX);
        msg.status.service = static_cast<uint16_t>(NavSatStatusData::Service::SERVICE_NONE);
    }
}

void OdomToVrtkMsg(const fixposition::FP_ODOMETRY& data, fixposition_driver_ros1::VRTK& msg) {
    if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.odom.stamp));
    }
    
    msg.header.frame_id = data.odom.frame_id;
    msg.pose_frame = data.odom.child_frame_id;
    msg.kin_frame = data.odom.child_frame_id;

    PoseWithCovDataToMsg(data.odom.pose, msg.pose);
    TwistWithCovDataToMsg(data.odom.twist, msg.velocity);
    tf::vectorEigenToMsg(data.acceleration, msg.acceleration);
    msg.fusion_status = data.fusion_status;
    msg.imu_bias_status = data.imu_bias_status;
    msg.gnss1_status = data.gnss1_status;
    msg.gnss2_status = data.gnss2_status;
    msg.wheelspeed_status = data.wheelspeed_status;
    msg.version = data.version;
}

void OdomToImuMsg(const fixposition::FP_ODOMETRY& data, sensor_msgs::Imu& msg) {
    if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.odom.stamp));
    }
    
    msg.header.frame_id = data.odom.frame_id;
    tf::vectorEigenToMsg(data.acceleration, msg.linear_acceleration);
    tf::vectorEigenToMsg(data.odom.twist.angular, msg.angular_velocity);
}

void VrtkDataToMsg(const VrtkData& data, fixposition_driver_ros1::VRTK& msg) {
    if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    }
    
    msg.header.frame_id = data.frame_id;
    msg.pose_frame = data.pose_frame;
    msg.kin_frame = data.kin_frame;

    PoseWithCovDataToMsg(data.pose, msg.pose);
    TwistWithCovDataToMsg(data.velocity, msg.velocity);
    tf::vectorEigenToMsg(data.acceleration, msg.acceleration);
    msg.fusion_status = data.fusion_status;
    msg.imu_bias_status = data.imu_bias_status;
    msg.gnss1_status = data.gnss1_status;
    msg.gnss2_status = data.gnss2_status;
    msg.wheelspeed_status = data.wheelspeed_status;
    msg.version = data.version;
}

void TfDataToMsg(const TfData& data, geometry_msgs::TransformStamped& msg) {
    msg.header.frame_id = data.frame_id;
    msg.child_frame_id = data.child_frame_id;

    if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    }

    tf::quaternionEigenToMsg(data.rotation, msg.transform.rotation);
    tf::vectorEigenToMsg(data.translation, msg.transform.translation);
}

}  // namespace fixposition
