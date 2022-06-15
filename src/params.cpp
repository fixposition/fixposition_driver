/**
 *  @file
 *  @brief Implementation of Parameter Loading
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

/* ROS */
#include <ros/console.h>
#include <ros/ros.h>

/* PACKAGE */
#include <fixposition_driver/params.hpp>

namespace fixposition {

bool FpOutputParams::LoadFromRos(const std::string &ns) {
    // read parameters
    if (!ros::param::get(ns + "/rate", rate)) rate = 100;

    std::string type_str;
    if (!ros::param::get(ns + "/type", type_str)) {
        return false;
    }
    if (type_str == "tcp") {
        type = INPUT_TYPE::TCP;
    } else if (type_str == "serial") {
        type = INPUT_TYPE::SERIAL;
    } else {
        ROS_ERROR("Input type has to be tcp or serial!");
        return false;
    }

    if (!ros::param::get(ns + "/formats", formats)) formats = {"ODOMETRY", "LLH", "RAWIMU", "CORRIMU", "TF"};

    // Get parameters: port (required)
    if (!ros::param::get(ns + "/port", port)) {
    }

    if (type == INPUT_TYPE::TCP) {
        if (!ros::param::get(ns + "/ip", ip)) {
            // default value for IP
            ip = "10.0.2.1";
        }
        if (!ros::param::get(ns + "/port", port)) {
            // default value for TCP port
            port = "21000";
        }
    } else if (type == INPUT_TYPE::SERIAL) {
        if (!ros::param::get(ns + "/baudrate", baudrate)) {
            // default value for baudrate
            baudrate = 115200;
        }
        if (!ros::param::get(ns + "/port", port)) {
            // default value for serial port
            port = "/dev/ttyUSB0";
        }
    }

    return true;
}

bool CustomerInputParams::LoadFromRos(const std::string &ns) {
    if (!ros::param::get(ns + "/speed_topic", speed_topic)) {
        // default value for the topic name
        speed_topic = "/fixposition/speed";
    }

    return true;
}

bool FixpositionDriverParams::LoadFromRos(const std::string &ns) {
    bool ok = true;

    ok &= fp_output.LoadFromRos(ns + "fp_output");
    ok &= customer_input.LoadFromRos(ns + "customer_input");

    return ok;
}

}  // namespace fixposition
