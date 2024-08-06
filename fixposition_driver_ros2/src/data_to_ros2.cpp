/**
 *  @file
 *  @brief Convert Data classes to ROS2 msgs
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
#include <fixposition_driver_ros2/data_to_ros2.hpp>

namespace fixposition {

void FpToRosMsg(const fixposition::OdometryData& data, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        nav_msgs::msg::Odometry msg;
        
        // Populate message
        if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.stamp);
        }
        msg.header.frame_id = data.frame_id;
        msg.child_frame_id = data.child_frame_id;

        PoseWithCovDataToMsg(data.pose, msg.pose);
        TwistWithCovDataToMsg(data.twist, msg.twist);

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const ImuData& data, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        sensor_msgs::msg::Imu msg;
        
        // Populate message
        if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.stamp);
        }
        msg.header.frame_id = data.frame_id;

        tf2::toMsg(data.linear_acceleration, msg.linear_acceleration);
        tf2::toMsg(data.angular_velocity, msg.angular_velocity);

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const FP_GNSSANT& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GNSSANT>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GNSSANT msg;
        
        // Populate message
        if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.stamp);
        }

        msg.gnss1_state = data.gnss1_state;
        msg.gnss1_power = data.gnss1_power;
        msg.gnss1_age = data.gnss1_age;
        msg.gnss2_state = data.gnss2_state;
        msg.gnss2_power = data.gnss2_power;
        msg.gnss2_age = data.gnss2_age;
        
        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const FP_GNSSCORR& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GNSSCORR>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GNSSCORR msg;
        
        // Populate message
        if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.stamp);
        }

        msg.gnss1_fix = data.gnss1_fix;
        msg.gnss1_nsig_l1 = data.gnss1_nsig_l1;
        msg.gnss1_nsig_l2 = data.gnss1_nsig_l2;
        msg.gnss2_fix = data.gnss2_fix;
        msg.gnss2_nsig_l1 = data.gnss2_nsig_l1;
        msg.gnss2_nsig_l2 = data.gnss2_nsig_l2;

        msg.corr_latency = data.corr_latency;
        msg.corr_update_rate = data.corr_update_rate;
        msg.corr_data_rate = data.corr_data_rate;
        msg.corr_msg_rate = data.corr_msg_rate;

        msg.sta_id = data.sta_id;
        tf2::toMsg(data.sta_llh, msg.sta_llh);
        msg.sta_dist = data.sta_dist;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const FP_LLH& data, rclcpp::Publisher<fixposition_driver_ros2::msg::LLH>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::LLH msg;

        // Populate message
        if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.stamp);
        }

        tf2::toMsg(data.llh, msg.position);
        Eigen::Map<Eigen::Matrix3d> cov_map = Eigen::Map<Eigen::Matrix3d>(msg.covariance.data());
        cov_map = data.cov;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const FP_ODOMENU& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMENU>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::ODOMENU msg;

        // Populate message
        if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.odom.stamp);
        }
        
        msg.header.frame_id = data.odom.frame_id;
        msg.pose_frame = data.odom.child_frame_id;
        msg.kin_frame = data.odom.child_frame_id;

        PoseWithCovDataToMsg(data.odom.pose, msg.pose);
        TwistWithCovDataToMsg(data.odom.twist, msg.velocity);
        tf2::toMsg(data.acceleration, msg.acceleration);
        msg.fusion_status = data.fusion_status;
        msg.imu_bias_status = data.imu_bias_status;
        msg.gnss1_status = data.gnss1_status;
        msg.gnss2_status = data.gnss2_status;
        msg.wheelspeed_status = data.wheelspeed_status;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const FP_ODOMETRY& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMETRY>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::ODOMETRY msg;

        // Populate message
        if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.odom.stamp);
        }
        
        msg.header.frame_id = data.odom.frame_id;
        msg.pose_frame = data.odom.child_frame_id;
        msg.kin_frame = data.odom.child_frame_id;

        PoseWithCovDataToMsg(data.odom.pose, msg.pose);
        TwistWithCovDataToMsg(data.odom.twist, msg.velocity);
        tf2::toMsg(data.acceleration, msg.acceleration);
        msg.fusion_status = data.fusion_status;
        msg.imu_bias_status = data.imu_bias_status;
        msg.gnss1_status = data.gnss1_status;
        msg.gnss2_status = data.gnss2_status;
        msg.wheelspeed_status = data.wheelspeed_status;
        msg.version = data.version;
        
        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const FP_ODOMSH& data, rclcpp::Publisher<fixposition_driver_ros2::msg::ODOMSH>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::ODOMSH msg;

        // Populate message
        if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.odom.stamp);
        }
        
        msg.header.frame_id = data.odom.frame_id;
        msg.pose_frame = data.odom.child_frame_id;
        msg.kin_frame = data.odom.child_frame_id;

        PoseWithCovDataToMsg(data.odom.pose, msg.pose);
        TwistWithCovDataToMsg(data.odom.twist, msg.velocity);
        tf2::toMsg(data.acceleration, msg.acceleration);
        msg.fusion_status = data.fusion_status;
        msg.imu_bias_status = data.imu_bias_status;
        msg.gnss1_status = data.gnss1_status;
        msg.gnss2_status = data.gnss2_status;
        msg.wheelspeed_status = data.wheelspeed_status;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const FP_TEXT& data, rclcpp::Publisher<fixposition_driver_ros2::msg::TEXT>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::TEXT msg;

        // Populate message
        msg.level = data.level;
        msg.text = data.text;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const GP_GGA& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPGGA>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GPGGA msg;

        // Populate message
        msg.time = data.time_str;
        msg.latitude = data.llh(0);
        msg.lat_ns = data.lat_ns;
        msg.longitude = data.llh(1);
        msg.lon_ew = data.lon_ew;
        msg.quality = data.quality;
        msg.num_sv = data.num_sv;
        msg.hdop = data.hdop;
        msg.alt = data.llh(2);
        msg.alt_unit = data.alt_unit;
        msg.diff_age = data.diff_age;
        msg.diff_sta = data.diff_sta;
        msg.sentence = data.sentence;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const GP_GLL& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPGLL>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GPGLL msg;

        // Populate message
        msg.latitude = data.latlon(0);
        msg.lat_ns = data.lat_ns;
        msg.longitude = data.latlon(1);
        msg.lon_ew = data.lon_ew;
        msg.time = data.time_str;
        msg.status = data.status;
        msg.mode = data.mode;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const GN_GSA& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GNGSA>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GNGSA msg;

        // Populate message
        msg.mode_op = data.mode_op;
        msg.mode_nav = data.mode_nav;
        
        for (unsigned int i = 0; i < data.ids.size(); i++) {
           msg.ids.push_back(data.ids.at(i));
        }
        
        msg.pdop = data.pdop;
        msg.hdop = data.hdop;
        msg.vdop = data.vdop;
        msg.gnss_id = data.gnss_id;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const GP_GST& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPGST>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GPGST msg;

        // Populate message
        msg.time = data.time_str;
        msg.rms_range = data.rms_range;
        msg.std_major = data.std_major;
        msg.std_minor = data.std_minor;
        msg.angle_major = data.angle_major;
        msg.std_lat = data.std_lat;
        msg.std_lon = data.std_lon;
        msg.std_alt = data.std_alt;
        
        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const GX_GSV& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GXGSV>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GXGSV msg;

        // Populate message
        msg.sentences = data.sentences;
        msg.sent_num = data.sent_num;
        msg.num_sats = data.num_sats;
        
        for (unsigned int i = 0; i < data.sat_id.size(); i++) {
            msg.sat_id.push_back(data.sat_id.at(i));
            msg.elev.push_back(data.elev.at(i));
            msg.azim.push_back(data.azim.at(i));
            msg.cno.push_back(data.cno.at(i));
        }

        msg.signal_id = data.signal_id;

        // Publish message
        pub->publish(msg);
    }
}


void FpToRosMsg(const GP_HDT& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPHDT>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GPHDT msg;

        // Populate message
        msg.heading = data.heading;
        msg.true_ind = data.true_ind;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const GP_RMC& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPRMC>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GPRMC msg;

        // Populate message
        msg.time = data.time_str;
        msg.status = data.status;
        msg.latitude = data.latlon(0);
        msg.lat_ns = data.lat_ns;
        msg.longitude = data.latlon(1);
        msg.lon_ew = data.lon_ew;
        msg.speed = data.speed;
        msg.course = data.course;
        msg.date = data.date_str;
        msg.mode = data.mode;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const GP_VTG& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPVTG>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GPVTG msg;

        // Populate message
        msg.cog_true = data.cog_true;
        msg.cog_ref_t = data.cog_ref_t;
        msg.cog_mag = data.cog_mag;
        msg.cog_ref_m = data.cog_ref_m;
        msg.sog_knot = data.sog_knot;
        msg.sog_unit_n = data.sog_unit_n;
        msg.sog_kph = data.sog_kph;
        msg.sog_unit_k = data.sog_unit_k;
        msg.mode = data.mode;

        // Publish message
        pub->publish(msg);
    }
}

void FpToRosMsg(const GP_ZDA& data, rclcpp::Publisher<fixposition_driver_ros2::msg::GPZDA>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        fixposition_driver_ros2::msg::GPZDA msg;

        // ROS Header
        if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.stamp);
        }
        msg.header.frame_id = "FP_POI";

        // Populate message
        msg.time = data.time_str;
        msg.date = data.date_str;
        msg.local_hr = data.local_hr;
        msg.local_min = data.local_min;

        // Publish message
        pub->publish(msg);
    }
}

void TfDataToMsg(const TfData& data, geometry_msgs::msg::TransformStamped& msg) {
    msg.header.frame_id = data.frame_id;
    msg.child_frame_id = data.child_frame_id;

    if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
        msg.header.stamp = rclcpp::Clock().now();
    } else {
        msg.header.stamp = GpsTimeToMsgTime(data.stamp);
    }

    msg.transform.rotation = tf2::toMsg(data.rotation);
    tf2::toMsg(data.translation, msg.transform.translation);
}

void NavSatFixDataToMsg(const NavSatFixData& data, sensor_msgs::msg::NavSatFix& msg) {
    if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
        msg.header.stamp = rclcpp::Clock().now();
    } else {
        msg.header.stamp = GpsTimeToMsgTime(data.stamp);
    }
    msg.header.frame_id = data.frame_id;
    msg.status.status = data.status.status;
    msg.status.service = data.status.service;
    msg.latitude = data.latitude;
    msg.longitude = data.longitude;
    msg.altitude = data.altitude;

    Eigen::Map<Eigen::Matrix<double, 3, 3>> cov_map =
        Eigen::Map<Eigen::Matrix<double, 3, 3>>(msg.position_covariance.data());
    cov_map = data.cov;

    msg.position_covariance_type = data.position_covariance_type;
}

void PoseWithCovDataToMsg(const PoseWithCovData& data, geometry_msgs::msg::PoseWithCovariance& msg) {
    msg.pose.position = tf2::toMsg(data.position);
    msg.pose.orientation = tf2::toMsg(data.orientation);

    Eigen::Map<Eigen::Matrix<double, 6, 6>> cov_map = Eigen::Map<Eigen::Matrix<double, 6, 6>>(msg.covariance.data());
    cov_map = data.cov;
}

void TwistWithCovDataToMsg(const fixposition::TwistWithCovData& data, geometry_msgs::msg::TwistWithCovariance& msg) {
    tf2::toMsg(data.linear, msg.twist.linear);
    tf2::toMsg(data.angular, msg.twist.angular);

    Eigen::Map<Eigen::Matrix<double, 6, 6>> cov_map = Eigen::Map<Eigen::Matrix<double, 6, 6>>(msg.covariance.data());
    cov_map = data.cov;
}

void OdometryDataToTf(const FP_ODOMETRY& data, std::shared_ptr<tf2_ros::TransformBroadcaster> pub) {
    if (data.fusion_status > 0) {
        // Ensure Fusion orientation is a valid quaternion
        if (data.odom.pose.orientation.vec().isZero() && (data.odom.pose.orientation.w() == 0)) {
            return;
        }

        // Create message
        geometry_msgs::msg::TransformStamped msg;

        // Populate message
        msg.header.frame_id = data.odom.frame_id;
        msg.child_frame_id = data.odom.child_frame_id;

        if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.odom.stamp);
        }

        msg.transform.rotation = tf2::toMsg(data.odom.pose.orientation);
        tf2::toMsg(data.odom.pose.position, msg.transform.translation);

        // Publish message
        pub->sendTransform(msg);
    }
}

void OdomToNavSatFix(const fixposition::FP_ODOMETRY& data, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        sensor_msgs::msg::NavSatFix msg;
    
        // Populate message header
        if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.odom.stamp);
        }
        msg.header.frame_id = data.odom.frame_id;
        
        // Populate LLH position
        const Eigen::Vector3d llh_pos = TfWgs84LlhEcef(data.odom.pose.position);
        msg.latitude  = RadToDeg(llh_pos(0));
        msg.longitude = RadToDeg(llh_pos(1));
        msg.altitude  = llh_pos(2);

        // Populate LLH covariance
        const Eigen::Matrix3d p_cov_e = data.odom.pose.cov.topLeftCorner(3, 3);
        const Eigen::Matrix3d C_l_e = RotEnuEcef(data.odom.pose.position);
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

        // Publish message
        pub->publish(msg);
    }
}

void OdomToImuMsg(const fixposition::FP_ODOMETRY& data, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        sensor_msgs::msg::Imu msg;
    
        // Populate message
        if (data.odom.stamp.tow == 0.0 && data.odom.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.odom.stamp);
        }
        
        msg.header.frame_id = data.odom.frame_id;
        tf2::toMsg(data.acceleration, msg.linear_acceleration);
        tf2::toMsg(data.odom.twist.angular, msg.angular_velocity);

        // Publish message
        pub->publish(msg);
    }
}

void OdomToYprMsg(const OdometryData& data, rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub) {
    if (pub->get_subscription_count() > 0) {
        // Create message
        geometry_msgs::msg::Vector3Stamped msg;

        // Populate message
        if (data.stamp.tow == 0.0 && data.stamp.wno == 0) {
            msg.header.stamp = rclcpp::Clock().now();
        } else {
            msg.header.stamp = GpsTimeToMsgTime(data.stamp);
        }
        msg.header.frame_id = "FP_ENU";

        // Euler angle wrt. ENU frame in the order of Yaw Pitch Roll
        Eigen::Vector3d enu_euler = RotToEul(data.pose.orientation.toRotationMatrix());
        msg.vector.set__x(enu_euler.x());
        msg.vector.set__y(enu_euler.y());
        msg.vector.set__z(enu_euler.z());

        // Publish message
        pub->publish(msg);
    }
}

}  // namespace fixposition
