/**
 * @file Uart.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Provides communication between the hardware PCB and the raspberry.
 * @version 0.1
 * @date 2020-01-03
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

// Standard library
#include <vector>

// ROS libraries
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

// Noah libraries

namespace noah_drivers {

class Uart {
   public:
    static constexpr const char *PACKAGE_NAME{"UART_DRIVER"};
    static constexpr const char *TOPIC_NAME{"uart_port"};

    /**
     * @brief Constructor
     */
    Uart(const std::string &serialPort);

    /**
     * @brief Destroy the Uart object
     */
    ~Uart();

    /**
     * @brief Opens the port
     *
     * @param serialPort name of the port to open connection.
     * @return true when open was successfull, false otherwise.
     */
    bool openPort();

    /**
     * @brief Close the port.
     *
     */
    bool closePort();

    /**
     * @brief Transmit data using USART.
     *
     * @param data data to be transmitted.
     * @return true data sent is OK. false elsewhere.
     */
    bool transmit(const std::vector<uint8_t> &data);

    /**
     * @brief Receive data using USART.
     *
     * @param size Amount of data to be read.
     * @return std::vector<uint8_t> data in buffer.
     */
    std::vector<uint8_t> receive(uint32_t size = 255);

    /**
     * @brief Checks if the port is already opened or not
     *
     * @return true when port is open, false elsewhere.
     */
    bool isOpen();

   private:
    /// @brief port name.
    const char *serial_port_name_;

    /**
     * @brief Stores if the port is open.
     */
    bool is_port_open_;

    /// @brief Status of the port.
    int serial_port_;
};
}  // namespace noah_drivers
