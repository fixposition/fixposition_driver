/**
 *  @file
 *  @brief Declaration of FixpositionDriver class
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

#ifndef __FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER__
#define __FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER__

/* SYSTEM / STL */
#include <termios.h>

#include <unordered_map>

/* EXTERNAL */

#include <fixposition_driver_lib/converter/base_converter.hpp>
#include <fixposition_driver_lib/fpb.hpp>
#include <fixposition_driver_lib/fpb_measurements.hpp>
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
    virtual bool RunOnce();

   protected:
    /**
     * @brief
     *
     * @param[in] sensors_meas vector of sensors, each sensor containing speed values
     */
    virtual void WsCallback(const std::vector<std::vector<int>>& sensors_meas);

    /**
     * @brief
     *
     * @param[in] meas_vec measurements from one specific wheelspeed sensor
     * @param[in] meas_loc location from the specific wheelspeed sensor
     * @param[out] meas_fpb fpb measurement to be filled from the vector
     * @return true if the measurement was successfully filled, false otherwise
     */
    virtual bool FillWsSensorMeas(const std::vector<int>& meas_vec, const FpbMeasurementsMeasLoc meas_loc,
                                  const FpbMeasurementsMeas meas_fpb);

    /**
     * @brief Convert the Nmea like string using correct converter
     *
     * @param[in] msg NMEA like string to be converted. $HEADER,,,,,,,*CHECKSUM
     */
    virtual void NmeaConvertAndPublish(const std::string& msg);

    /**
     * @brief Convert the buffer after identified as Nov msg
     *
     * @param[in] msg ptr to the start of the msg
     * @param[in] size size of the msg
     */
    virtual void NovConvertAndPublish(const uint8_t* msg, int size);

    /**
     * @brief Initialize convertes based on config
     *
     * @return true
     * @return false
     */
    virtual bool InitializeConverters();

    /**
     * @brief Read data and publish to ros if possible
     *
     * @return true data read success or no data
     * @return false connection problems, restart the connection
     */
    virtual bool ReadAndPublish();

    /**
     * @brief Connect the defined TCP or Serial socket
     *
     * @return true success
     * @return false cannot connect
     */
    virtual bool Connect();

    /**
     * @brief Initialize TCP connection
     *
     * @return true success
     * @return false fail
     */
    virtual bool CreateTCPSocket();

    /**
     * @brief Initialize Serial connection
     *
     * @return true success
     * @return false fail
     */
    virtual bool CreateSerialConnection();

    FixpositionDriverParams params_;

    RAWDMI rawdmi_;  //!< RAWDMI msg struct
    // struct FpbHeader fpb_header_;                  //!< fpb standard header
    // struct FpbMeasurementsHeader fpb_meas_header;  //!< fpb measurements specific header
    // // struct FpbMeasurementsMeas

    std::unordered_map<std::string, std::unique_ptr<BaseAsciiConverter>>
        a_converters_;  //!< ascii converters corresponding to the input formats

    using BestgnssposObserver = std::function<void(const Oem7MessageHeaderMem*, const BESTGNSSPOSMem*)>;
    std::vector<BestgnssposObserver> bestgnsspos_obs_;  //!< observers for bestgnsspos

    // TODO: Add more NOV types

    int client_fd_ = -1;  //!< TCP or Serial file descriptor
    int connection_status_ = -1;
    struct termios options_save_;
};
}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER__
