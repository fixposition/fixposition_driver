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
 * @brief Fixposition sensor driver
 */

#ifndef __FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER_HPP__
#define __FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER_HPP__

/* LIBC/STL */
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

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
    FixpositionDriver(const DriverParams& params);

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
    using NovbObserver = std::function<void(const fpsdk::common::parser::novb::NovbHeader*, const uint8_t*)>;

    //! Observer function for raw parser messages
    using RawObserver = std::function<void(const fpsdk::common::parser::ParserMsg&)>;

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
     * @brief Add observer for raw parser messages
     *
     * @param[in]  observer  The function to call to process the message
     */
    void AddRawObserver(RawObserver observer);

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

    /**
     * @brief Remove all observers for raw parser messages
     */
    void RemoveRawObservers();

    /**
     * @brief Send correction data to sensor
     *
     * @param[in]  msg   The raw message data (e.g. a RTCM3 message)
     * @param[in]  size  Size of the raw message
     */
    void SendCorrectionData(const uint8_t* msg, const std::size_t size);

    /**
     * @brief Send wheelspeed data to sensor
     *
     * @param[in]  data  The wheelspeed data
     */
    void SendWheelspeedData(const std::vector<WheelSpeedData>& data);

   protected:
    DriverParams params_;

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
    fpsdk::common::parser::Parser parser_;  //!< Protocol parser for incoming messages
    fpsdk::common::thread::Thread worker_;  //!< Worker thread handle
    bool Worker();                          //!< Worker thread

    // Observers for received messages
    std::unordered_map<std::string, std::vector<FpaObserver>> fpa_observers_;    //!< FP_A message observers
    std::unordered_map<std::string, std::vector<NmeaObserver>> nmea_observers_;  //!< NMEA message observers
    std::unordered_map<std::string, std::vector<NovbObserver>> novb_observers_;  //!< NOV_B message observers
    std::vector<RawObserver> raw_observers_;                                     //!< Raw parser message observers
    void NotifyFpaObservers(const fpsdk::common::parser::ParserMsg& msg);        //!< Notify FP_A message observers
    void NotifyNmeaObservers(const fpsdk::common::parser::ParserMsg& msg);       //!< Notify NMEA message observers
    void NotifyNovbObservers(const fpsdk::common::parser::ParserMsg& msg);       //!< Notify NOV_B message observers
    void NotifyRawObservers(const fpsdk::common::parser::ParserMsg& msg);        //!< Notify raw message observers
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_LIB_FIXPOSITION_DRIVER_HPP__
