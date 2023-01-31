/**
 *  @file
 *  @brief Convert Data classes to ROS1 msgs
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* PACKAGE */
#include <fixposition_driver_ros1/data_to_ros1.hpp>

namespace fixposition {
void ImuDataToMsg(const ImuData& data, sensor_msgs::Imu& msg) {
    msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    msg.header.frame_id = data.frame_id;
    tf::vectorEigenToMsg(data.linear_acceleration, msg.linear_acceleration);
    tf::vectorEigenToMsg(data.angular_velocity, msg.angular_velocity);
}

void NavSatFixDataToMsg(const NavSatFixData& data, sensor_msgs::NavSatFix& msg) {
    msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    msg.header.frame_id = data.frame_id;

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
    msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    msg.header.frame_id = data.frame_id;
    msg.child_frame_id = data.child_frame_id;

    PoseWithCovDataToMsg(data.pose, msg.pose);
    TwistWithCovDataToMsg(data.twist, msg.twist);
}

void VrtkDataToMsg(const VrtkData& data, fixposition_driver_ros1::VRTK& msg) {
    msg.header.stamp = ros::Time::fromBoost(GpsTimeToPtime(data.stamp));
    msg.header.frame_id = data.frame_id;
    msg.pose_frame = data.pose_frame;
    msg.kin_frame = data.kin_frame;

    PoseWithCovDataToMsg(data.pose, msg.pose);
    TwistWithCovDataToMsg(data.velocity, msg.velocity);
    tf::vectorEigenToMsg(data.acceleration, msg.acceleration);
    msg.fusion_status = data.fusion_status;
    msg.imu_bias_status = data.imu_bias_status;
    msg.gnss_status = data.gnss_status;
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
