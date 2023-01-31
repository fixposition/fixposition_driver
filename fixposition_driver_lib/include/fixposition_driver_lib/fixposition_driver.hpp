/**
 *  @file
 *  @brief Declaration of FixpositionDriver class
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER__
#define __FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER__

/* SYSTEM / STL */
#include <termios.h>

#include <unordered_map>

/* EXTERNAL */

#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/params.hpp>
#include <fixposition_driver_lib/rawdmi.hpp>

namespace fixposition {

class FixpositionDriver {
   public:
    /**
     * @brief Construct a new FixpositionDriver object
     *
     */
    FixpositionDriver(const FixpositionDriverParams& params);

    /**
     * @brief Destroy the Fixposition Driver object, close all open connections
     *
     */
    ~FixpositionDriver();

    /**
     * @brief Run in Loop the Read Convert and Publish cycle
     *
     */
    bool RunOnce();

   protected:
    /**
     * @brief
     *
     * @param[in] msg
     */
    void WsCallback(const std::vector<int>& speeds);

    /**
     * @brief Convert the string using correct converter
     *
     * @param[in] msg NMEA like string to be converted. $HEADER,,,,,,,*CHECKSUM
     */
    void ConvertAndPublish(const std::string& msg);

    /**
     * @brief Initialize convertes based on config
     *
     * @return true
     * @return false
     */
    bool InitializeConverters();

    /**
     * @brief Read data and publish to ros if possible
     *
     * @return true data read success or no data
     * @return false connection problems, restart the connection
     */
    bool ReadAndPublish();

    /**
     * @brief Connect the defined TCP or Serial socket
     *
     * @return true success
     * @return false cannot connect
     */
    bool Connect();

    /**
     * @brief Initialize TCP connection
     *
     * @return true success
     * @return false fail
     */
    bool CreateTCPSocket();

    /**
     * @brief Initialize Serial connection
     *
     * @return true success
     * @return false fail
     */
    bool CreateSerialConnection();

    FixpositionDriverParams params_;

    RAWDMI rawdmi_;  //!< RAWDMI msg struct

    std::unordered_map<std::string, std::unique_ptr<BaseConverter>>
        converters_;  //!< converters corresponding to the input formats

    int client_fd_ = -1;  //!< TCP or Serial file descriptor
    int connection_status_ = -1;
    struct termios options_save_;
};
}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER__
