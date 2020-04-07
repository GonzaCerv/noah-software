/**
 * @file PackageManager.cpp
 * @author Gonzalo Cervetti (cervetti.g@gamil.com)
 * @brief Manages incomming packages from UART.
 * @version 0.1
 * @date 2020-02-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

// Standard library
#include <functional>
#include <list>
#include <memory>
#include <queue>

// ROS libraries

// Noah libraries
#include "noah_drivers/Uart.hpp"
#include "noah_drivers/UartPackage.hpp"

namespace noah_drivers {

using ExecuteMissionCallback = std::function<void(const UartPackage &package)>;

class PackageManager {
   public:
    PackageManager(Uart uart_port, const ExecuteMissionCallback &execute_callback);

    ~PackageManager();

    /**
     * @brief Starts the port and tries to initiate the commnunication.
     *
     * @return true when everything is ok, false elsewhere.
     */
    bool start();

    /**
     * @brief Searchs for new incomming packages and process them.
     *        If a new package is present and it is valid, call the
     *        callback and passes the package to execute the corresponding task.
     *
     */
    void check();

    /**
     * @brief
     *
     * @param package Sends a package to the PCB.
     * @return true Everything goes great, else elsewhere.
     */
    bool sendPackage(UartPackage package);

   private:
    /**
     * @brief Get one package of data.
     *
     * @return new package to be processed. If there is no new package, it
     * returns a package with an EMPTY command.
     */
    std::list<UartPackage> getPackages();

    /**
     * @brief Name of the package for logging.
     */
    static constexpr const char *PACKAGE_NAME{"PCB_PACKAGE_MANAGER"};

    /// @brief Port used for communications.
    Uart uart_port_;

    /// @brief function will be called when new package is available for processing.
    ExecuteMissionCallback execute_callback_;

    /// @brief Buffer of packages available to execute.
    std::list<UartPackage> packages_;
};
}  // namespace noah_drivers