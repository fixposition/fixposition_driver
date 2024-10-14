/**
 *  @file
 *  @brief Implementation of Parameter Loading
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
#include <fixposition_driver_ros1/params.hpp>

namespace fixposition {

bool LoadParamsFromRos1(const std::string& ns, FpOutputParams& params) {
    const std::string RATE = ns + "/rate";
    const std::string RECONNECT_DELAY = ns + "/reconnect_delay";
    const std::string TYPE = ns + "/type";
    const std::string FORMATS = ns + "/formats";
    const std::string IP = ns + "/ip";
    const std::string PORT = ns + "/port";
    const std::string BAUDRATE = ns + "/baudrate";
    const std::string COV_WARNING = ns + "/cov_warning";
    const std::string NAV2_MODE = ns + "/nav2_mode";

    // read parameters
    if (!ros::param::get(RATE, params.rate)) {
        params.rate = 100;
        ROS_WARN("Using Default Rate : %d", params.rate);
    }
    if (!ros::param::get(RECONNECT_DELAY, params.reconnect_delay)) {
        params.reconnect_delay = 5.0;
        ROS_WARN("Using Default Reconnect Delay : %f", params.reconnect_delay);
    }
    if (!ros::param::get(COV_WARNING, params.cov_warning)) {
        params.cov_warning = false;
        ROS_WARN("Using Default Covariance Warning option : %i", params.cov_warning);
    }
    if (!ros::param::get(NAV2_MODE, params.nav2_mode)) {
        params.nav2_mode = false;
        ROS_WARN("Using Default Nav2 mode option : %i", params.nav2_mode);
    }

    std::string type_str;
    if (!ros::param::get(TYPE, type_str)) {
        return false;
    }
    if (type_str == "tcp") {
        params.type = INPUT_TYPE::TCP;
    } else if (type_str == "serial") {
        params.type = INPUT_TYPE::SERIAL;
    } else {
        ROS_ERROR("Input type has to be tcp or serial!");
        return false;
    }

    if (!ros::param::get(FORMATS, params.formats)) {
        params.formats = {"ODOMETRY", "RAWIMU", "CORRIMU", "TF"};
    }

    for (size_t i = 0; i < params.formats.size(); i++) {
        ROS_INFO("%s[%ld] : %s", FORMATS.c_str(), i, params.formats.at(i).c_str());
    }

    if (params.type == INPUT_TYPE::TCP) {
        if (!ros::param::get(IP, params.ip)) {
            // default value for IP
            params.ip = "10.0.2.1";
            ROS_WARN("Using Default IP : %s", params.ip.c_str());
        }
        if (!ros::param::get(PORT, params.port)) {
            // default value for TCP port
            params.port = "21000";
            ROS_WARN("Using Default Port : %s", params.port.c_str());
        }
        ROS_INFO("%s : %s", IP.c_str(), params.ip.c_str());
        ROS_INFO("%s : %s", PORT.c_str(), params.port.c_str());
    } else if (params.type == INPUT_TYPE::SERIAL) {
        if (!ros::param::get(BAUDRATE, params.baudrate)) {
            // default value for baudrate
            params.baudrate = 115200;
        }
        if (!ros::param::get(PORT, params.port)) {
            // default value for serial port
            params.port = "/dev/ttyUSB0";
        }
        ROS_INFO("%s : %d", BAUDRATE.c_str(), params.baudrate);
        ROS_INFO("%s : %s", PORT.c_str(), params.port.c_str());
    }

    return true;
}

bool LoadParamsFromRos1(const std::string& ns, CustomerInputParams& params) {
    const std::string SPEED_TOPIC = ns + "/speed_topic";
    if (!ros::param::get(SPEED_TOPIC, params.speed_topic)) {
        // default value for the topic name
        params.speed_topic = "/fixposition/speed";
        ROS_WARN("Using Default Speed Topic : %s", params.speed_topic.c_str());
    }
    ROS_INFO("%s : %s", SPEED_TOPIC.c_str(), params.speed_topic.c_str());

    const std::string RTCM_TOPIC = ns + "/rtcm_topic";
    if (!ros::param::get(RTCM_TOPIC, params.rtcm_topic)) {
        // default value for the topic name
        params.rtcm_topic = "/fixposition/rtcm";
        ROS_WARN("Using Default Rtcm Topic : %s", params.rtcm_topic.c_str());
    }
    ROS_INFO("%s : %s", RTCM_TOPIC.c_str(), params.rtcm_topic.c_str());

    return true;
}

bool LoadParamsFromRos1(const std::string& ns, FixpositionDriverParams& params) {
    bool ok = true;

    ok &= LoadParamsFromRos1(ns + "fp_output", params.fp_output);
    ok &= LoadParamsFromRos1(ns + "customer_input", params.customer_input);

    return ok;
}

}  // namespace fixposition
