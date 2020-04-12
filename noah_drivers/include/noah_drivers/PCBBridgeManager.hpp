/**
 * @file PCBBridgeManager.cpp
 * @author Gonzalo Cervetti (cervetti.g@gamil.com)
 * @brief Manages incomming packages from PCB using UART.
 * @version 0.1
 * @date 2020-02-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

// Standard library
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

// ROS libraries
#include "ros/ros.h"

// Noah libraries
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

#include "noah_drivers/UartPackage.hpp"

namespace noah_drivers {

using LibSerial::AlreadyOpen;
using LibSerial::SerialPort;

class PCBBridgeManager {
   public:
    PCBBridgeManager(const std::string &port_name, const std::chrono::milliseconds &check_interval_);

    ~PCBBridgeManager();

    /**
     * @brief Checks if there is new packages to process
     * 
     * @return List of packages to be processed
     */
    std::shared_ptr<UartPackage> check();

    /**
     * @brief
     *
     * @param package Sends a package to the PCB.
     * @return true Everything goes great, else elsewhere.
     */
    bool sendPackage(const UartPackage &package);

   private:
    /// @brief Name of the package for logging.
    static constexpr const char *PACKAGE_NAME_{"PCB_PACKAGE_MANAGER"};
    const uint16_t BUFFER_SIZE_{100};

    /**
     * @brief Decode all the packages found in the last incomming UART communication.
     * 
     * @param new_data raw data that comes from USART.
     * @return new package to be processed. If there is no new package, it
     * returns a package with an EMPTY command.
     */
    std::list<UartPackage> decodePackage(const std::vector<uint8_t> &new_data);

    /**
     * @brief Thread that checks incomming data to UART.
     *
     */
    void incommingPackagesThread();

    /// @brief Stops the execution of the thread.
    std::atomic<bool> halt_{false};

    /// @brief Port used for communications.
    SerialPort serial_port_;

    /// @brief Buffer of packages available to execute.
    std::list<UartPackage> packages_;

    /// @brief Stores the thread that checks incomming uart packages.
    std::unique_ptr<std::thread> incomming_packages_thread_;

    /// @brief Manages the accesing of the objects between thread.
    std::mutex mutex_;

    std::condition_variable cv_;

    std::chrono::milliseconds check_interval_;
};
}  // namespace noah_drivers