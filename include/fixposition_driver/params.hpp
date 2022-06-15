/**
 *  @file
 *  @brief Parameters
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_PARAMS_HPP__

/* SYSTEM / STL */
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */

namespace fixposition {

enum class INPUT_TYPE { TCP = 1, SERIAL = 2 };

struct FpOutputParams {
    int rate;                          //!< loop rate of the main read loop
    INPUT_TYPE type;                   //!< TCP or SERIAL
    std::vector<std::string> formats;  //!< data formats to convert, support "FP" and "LLH" for now

    std::string ip;    //!< IP address for TCP connection
    std::string port;  //!< Port for TCP connection
    int baudrate;      //!< baudrate of serial connection

    /**
     * @brief Load all parameters from ROS parameter server
     *
     * @param[in] ns namespace to load the parameters from
     * @return true success
     * @return false fail
     */
    bool LoadFromRos(const std::string &ns);
};
struct CustomerInputParams {
    std::string speed_topic;
    /**
     * @brief Load all parameters from ROS parameter server
     *
     * @param[in] ns namespace to load the parameters from
     * @return true success
     * @return false fail
     */
    bool LoadFromRos(const std::string &ns);
};

struct FixpositionDriverParams {
    FpOutputParams fp_output;
    CustomerInputParams customer_input;
    bool LoadFromRos(const std::string &ns = "~/");
};

}  // namespace fixposition

#endif
