/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Declaration of FixpositionDriver class
 */

#ifndef __FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER_HPP__
#define __FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER_HPP__

/* LIBC/STL */
#include <array>
#include <cstdint>
#include <functional>
#include <string>

/* EXTERNAL */
#include <poll.h>
#include <termios.h>

#include <boost/core/noncopyable.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/fpb.hpp>
#include <fpsdk_common/parser/nmea.hpp>
#include <fpsdk_common/parser/novb.hpp>
#include <fpsdk_common/thread.hpp>

/* PACKAGE */
#include "fixposition_driver_lib/helper.hpp"
#include "fixposition_driver_lib/messages/base_converter.hpp"
#include "fixposition_driver_lib/messages/fpa_type.hpp"
#include "fixposition_driver_lib/messages/nmea_type.hpp"
#include "fixposition_driver_lib/params.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Fixposition driver
 *
 * This handles the connection to a Fixposition Vision-RTK 2 sensor. It receives and decodes the received messages,
 * which can be used by consumers by observing ("subscribing to") the driver. On various events, such as receiving
 * a new message or completing and "epoch", the observers are notified by the driver. The driver also handles sending
 * data to the sensor, namely GNSS correction data (typically, RTCM3 messages) or measurements, such as wheelspeed.
 */
class FixpositionDriver : private boost::noncopyable {
   public:
    /**
     * @brief Constructor
     *
     * @params[in]  params  Parameters
     */
    FixpositionDriver(const SensorParams& params);

    /**
     * @brief Destructor
     */
    virtual ~FixpositionDriver();

    //
    // ----- Methods for starting and stopping the driver -----

    /**
     * @brief Connect to sensor and start the worker thread
     *
     * @returns true on success, false otherwise
     */
    bool StartDriver();

    /**
     * @brief Stop the worker thread and disconnect from sensor
     */
    void StopDriver();

    //
    // ----- Methods for observing messages received from the sensor -----

    //! Observer function for FP_A messages
    using FpaObserver = std::function<void(const fpsdk::common::parser::fpa::FpaPayload&)>;

    //! Observer function for NMEA messages
    using NmeaObserver = std::function<void(const fpsdk::common::parser::nmea::NmeaPayload&)>;

    //! Observer function for NOV_B messages
    using NovbObserver = std::function<void(const fpsdk::common::parser::ParserMsg&)>;

    /**
     * @brief Add observer for FP_A message
     *
     * @param[in]  message_name   The message name to observe, e.g. "FP_A-ODOMETRY"
     * @param[in]  observer       The function to call to process the message
     */
    void AddFpaObserver(const std::string& message_name, FpaObserver observer);

    /**
     * @brief Add observer for NMEA message
     *
     * @param[in]  formatter  The formatter for the messages to observe, e.g. "GGA"
     * @param[in]  observer   The function to call to process the message
     */
    void AddNmeaObserver(const std::string& formatter, NmeaObserver observer);

    /**
     * @brief Add observer for NOV_B message
     *
     * @param[in]  message_name   The message name to observe, e.g. "NOV_B-BESTGNSSPOS"
     * @param[in]  observer       The function to call to process the message
     */
    void AddNovbObserver(const std::string& message_name, NovbObserver observer);

    /**
     * @brief Remove all observers for FP_A messages
     */
    void RemoveFpaObservers();

    /**
     * @brief Remove all observers for NMEA messages
     */
    void RemoveNmeaObservers();

    /**
     * @brief Remove all observers for NOV_B messages
     */
    void RemoveNovbObservers();

   protected:
    SensorParams params_;

    // ---- Methods to handle the sensor connection ----

    /**
     * @brief Connect to sensor
     *
     * @returns true on success, false otherwise
     */
    virtual bool Connect();

    /**
     * @brief Disconnect from sensor
     *
     * @returns true on success, false otherwise
     */
    virtual void Disconnect();

    /**
     * @brief Check connection
     *
     * @returns true if connection is up, false otherwise
     */
    virtual bool IsConnected() const;

    /**
     * @brief Read data
     *
     * @param[out]  buf      Buffer to store read data
     * @param[in]   size     Size of buffer
     * @param[in]   timeout  Timeout [ms] to wait for data
     *
     * @returns the size read
     */
    virtual std::size_t Read(uint8_t* buf, const std::size_t size, const int timeout);

    /**
     * @brief Write data
     *
     * @param[in]  bu f  Buffer to the data
     * @param[in]  size  Size of the data
     *
     * @returns true if the data was sent, false otherwise
     */
    virtual bool Write(const uint8_t* buf, const std::size_t size);

    //
    //
    //
    //
    //
    //
    // TODO...
    //

    /**
     * @brief
     *
     * @param[in] sensors_meas map wheelspeed of sensors, each sensor containing speed values and their validity flag
     */
    void WsCallback(const std::unordered_map<std::string, std::vector<std::pair<bool, int>>>& sensors_meas);

    /**
     * @brief
     *
     * @param[in] rtcm_msg string with ASCII data for RTK correction
     */
    void RtcmCallback(const uint8_t* rtcm_msg, const size_t msg_size);

    std::unordered_map<std::string, std::unique_ptr<BaseAsciiConverter>>
        a_converters_;  //!< ascii converters corresponding to the input formats

    using BestgnssposObserver = std::function<void(const fpsdk::common::parser::novb::NovbLongHeader*,
                                                   const fpsdk::common::parser::novb::NovbBestgnsspos*)>;
    std::vector<BestgnssposObserver> bestgnsspos_obs_;  //!< observers for bestgnsspos

    // TODO: Add more NOV types

   private:
    // Sensor connection
    bool ConnectTcp(const std::string& ip, const int port);      //!< Connect() for TCP
    void DisconnectTcp();                                        //!< Disconnect() for TCP
    bool ConnectSerial(const std::string& dev, const int baud);  //!< Connect() for serial
    void DisconnectSerial();                                     //!< Disconnect() for serial
    bool serial_ = false;                                        //!< Serial (true) or TCP client (false)
    int sensor_fd_ = -1;                                         //!< TCP or serial file descriptor
    std::string sensor_name_;                                    //!< Name of the sensor, for debugging
    std::array<struct pollfd, 1> poll_fds_;                      //!< poll() config
    struct termios options_save_;                                //!< Saved serial port parameters

    // Worker thread
    fpsdk::common::parser::Parser parser_;                          //!< Protocol parser for incoming messages
    fpsdk::common::thread::Thread worker_;                          //!< Worker thread handle
    void Worker(void* arg);                                         //!< Worker thread
    void ProcessFpa(const fpsdk::common::parser::ParserMsg& msg);   //!< Process FP_A message
    void ProcessNmea(const fpsdk::common::parser::ParserMsg& msg);  //!< Process NMEA message
    void ProcessNovb(const fpsdk::common::parser::ParserMsg& msg);  //!< Process NOV_B message

    std::unordered_map<std::string, std::vector<FpaObserver>> fpa_observers_;
    std::unordered_map<std::string, std::vector<NmeaObserver>> nmea_observers_;
    std::unordered_map<std::string, std::vector<NovbObserver>> novb_observers_;

    // TODO: replace these
    void NmeaConvertAndPublish(const fpsdk::common::parser::ParserMsg& msg);
    void NovConvertAndPublish(const fpsdk::common::parser::ParserMsg& msg);
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER_HPP__
