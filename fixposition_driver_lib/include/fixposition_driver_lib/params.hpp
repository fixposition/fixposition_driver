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

/* EXTERNAL */

/* PACKAGE */

namespace fixposition {

enum class INPUT_TYPE { TCP = 1, SERIAL = 2 };

struct FpOutputParams {
    int rate;                          //!< loop rate of the main read loop
    double reconnect_delay;            //!< wait time in [s] until retry connection
    INPUT_TYPE type;                   //!< TCP or SERIAL
    std::vector<std::string> formats;  //!< data formats to convert, support "FP" and "LLH" for now

    std::string ip;    //!< IP address for TCP connection
    std::string port;  //!< Port for TCP connection
    int baudrate;      //!< baudrate of serial connection
};
struct CustomerInputParams {
    std::string speed_topic;
};

struct FixpositionDriverParams {
    FpOutputParams fp_output;
    CustomerInputParams customer_input;
};

}  // namespace fixposition

#endif
