/**
 *  @file
 *  @brief Parameters
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

#ifndef __FIXPOSITION_DRIVER_LIB_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_LIB_PARAMS_HPP__

/* SYSTEM / STL */
#include <string>
#include <vector>

namespace fixposition {

enum class INPUT_TYPE { TCP = 1, SERIAL = 2 };

struct FpOutputParams {
    int rate;                          //!< loop rate of the main read loop
    double reconnect_delay;            //!< wait time in [s] until retry connection
    INPUT_TYPE type;                   //!< TCP or SERIAL
    std::vector<std::string> formats;  //!< data formats to convert, supports "FP" for now
    std::string qos_type;              //!< ROS QoS type, supports "sensor_<short/long>" and "default_<short/long>"
    bool cov_warning;                  //!< enable/disable covariance warning
    bool nav2_mode;                    //!< enable/disable nav2 mode

    std::string ip;    //!< IP address for TCP connection
    std::string port;  //!< Port for TCP connection
    int baudrate;      //!< baudrate of serial connection
};
struct CustomerInputParams {
    std::string speed_topic;  //!< Input ROS topic for Speed measurements
    std::string rtcm_topic;   //!< Input ROS topic for RTCM3 messages
};

struct FixpositionDriverParams {
    FpOutputParams fp_output;
    CustomerInputParams customer_input;
};

}  // namespace fixposition

#endif
