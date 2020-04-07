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

// Standard library
#include <memory>

// ROS libraries

// Noah libraries
#include "noah_drivers/PackageManager.hpp"

namespace noah_drivers {
PackageManager::PackageManager(Uart uart_port, const ExecuteMissionCallback &execute_callback)
    : uart_port_(uart_port), execute_callback_(execute_callback) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // Stop the node if it isn't able to start the Uart connection.
    if (!uart_port_.openPort()) ros::shutdown();
}
PackageManager::~PackageManager() {}

bool PackageManager::start() {
    if (uart_port_.isOpen()) {
        // Send a single package to check that PCB is present.
        std::vector<uint8_t> package = {UartPackage::startPackage(), UartPackage::commandToValue(UartCommand::ECHO_CMD),
                                        UartPackage::stopPackage()};
        uart_port_.transmit(package);
        ros::Duration(0.5).sleep();

        // Leemos se recibieron nuevos paquetes.
        auto received_data = uart_port_.receive();
        if (received_data.empty()) {
            ROS_WARN_STREAM_NAMED(PACKAGE_NAME,
                                  "No response from PCB. Node will still run but no connection is assured.");
            return false;
        }
        return true;
    }
    return false;
}

void PackageManager::check() {
    // Check if there was some packages left. If there
    // was any, try to search new incomming packages.
    if (packages_.empty()) packages_ = getPackages();

    // If still empty, return.
    if (packages_.empty()) return;

    // If not, execute one more package.
    execute_callback_(packages_.front());
    packages_.pop_front();
    return;
}

std::list<UartPackage> PackageManager::getPackages() {
    auto result = std::list<UartPackage>();
    // Check if there are new bytes.
    auto new_bytes = uart_port_.receive();
    if (new_bytes.empty()) {
        return result;
    }
    while (!new_bytes.empty()) {
        // Search a valid start package to start processing.
        auto it = std::find(new_bytes.begin(), new_bytes.end(), UartPackage::startPackage());
        if (it == new_bytes.end()) return result;

        // Pops elements until it finds the start package byte.
        while (new_bytes.front() != UartPackage::startPackage()) {
            new_bytes.erase(new_bytes.begin());
        }
        // Pops the start package
        new_bytes.erase(new_bytes.begin());

        // Get the command and checks if it is a valid command. If that's not the case,
        // jump and search the next package.
        if (UartPackage::valueToCommand(new_bytes.front()) == UartCommand::ERROR) continue;
        UartPackage current_package(UartPackage::valueToCommand(new_bytes.front()));
        new_bytes.erase(new_bytes.begin());

        // Get the params.
        auto param_count = (UartPackage::commandToValue(current_package.command_) & 0xF0) >> 4;
        std::vector<uint8_t> params;
        while ((param_count != 0) && (!new_bytes.empty())) {
            params.push_back(new_bytes.front());
            new_bytes.erase(new_bytes.begin());
            --param_count;
        }
        // If there is no stopPackage at the end of the package, it searches the
        // next package.
        if (new_bytes.front() != UartPackage::stopPackage()) {
            continue;
        }
        current_package.parameters_ = params;
        result.push_back(current_package);
    }
    return result;
}

bool PackageManager::sendPackage(UartPackage package) {
    if (uart_port_.isOpen()) {
        std::vector<uint8_t> buffer;
        buffer.emplace_back(package.getCommandValue());

        for (auto i : package.parameters_) {
            buffer.emplace_back(i);
        }

        buffer.push_back(UartPackage::stopPackage());
        uart_port_.transmit(buffer);
        return true;
    }
    return false;
}

}  // namespace noah_drivers
