/**
 * @file PCBBridgeManager.cpp
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
#include "noah_drivers/PCBBridgeManager.hpp"

namespace noah_drivers {
PCBBridgeManager::PCBBridgeManager(const std::string &port_name, const std::chrono::milliseconds &check_interval) :
    check_interval_(check_interval){
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // Stop the node if it isn't able to start the Uart connection.
    try {
        serial_port_.Open(port_name);
    } catch (...) {
        ROS_ERROR_STREAM_NAMED(PACKAGE_NAME_, "Unable to open serial port");
        ros::shutdown();
    }
    // Set the baud rate of the serial port.
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    // Set the number of data bits.
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);

    // Turn off hardware flow control.
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    // Disable parity.
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);

    // Set the number of stop bits.
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    incomming_packages_thread_ =
        std::make_unique<std::thread>(std::bind(&PCBBridgeManager::incommingPackagesThread, this));
}

PCBBridgeManager::~PCBBridgeManager() {
    halt_ = true;
    cv_.notify_all();
    serial_port_.Close();
    incomming_packages_thread_->join();
}

std::shared_ptr<UartPackage> PCBBridgeManager::check() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::shared_ptr<UartPackage> pkg;
    if (!packages_.empty()){
        // The front is sotred in new_pkg so the reference of the object is not deleted by 'pop_front'
        auto new_pkg = packages_.front();
        pkg = std::make_shared<UartPackage>(new_pkg);
        packages_.pop_front();
    }
    cv_.notify_all();
    return pkg;
}

bool PCBBridgeManager::sendPackage(const UartPackage &package) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool result = false;
    if (serial_port_.IsOpen()) {
        std::vector<uint8_t> buffer;
        buffer.emplace_back(UartPackage::startPackage());
        buffer.emplace_back(UartPackage::commandToValue(package.getCommand()));
        for(auto param : package.getData()){
            buffer.emplace_back(param);
        }
        buffer.emplace_back(UartPackage::stopPackage());
        serial_port_.Write(buffer);
        result = true;
    }
    cv_.notify_all();
    return result;
}

std::list<UartPackage> PCBBridgeManager::decodePackage(const std::vector<uint8_t> &new_data) {
    auto result = std::list<UartPackage>();
    // Check if there are new bytes.
    if (new_data.empty()) {
        return result;
    }

    auto it = new_data.begin();
    UartPackage current_package;

    while (it != new_data.end()) {
        // Search a valid start package to start processing. 
        // If there is no start package, then iterate and exit the while.
        it = std::find(it, new_data.end(), UartPackage::startPackage());
        if (it == new_data.end()) continue;

        // Get the command and checks if it is a valid command. If that's not the case,
        // jump and search the next package.
        ++it;
        auto command = UartPackage::valueToCommand(*it);
        if (command == UartCommand::ERROR) continue;
        // Get the params.
        ++it;
        auto param_count = (UartPackage::commandToValue(command) & 0xF0) >> 4;
        std::vector<uint8_t> params;
        while ((param_count != 0) && (it != new_data.end())) {
            params.emplace_back(*it);
            ++it;
            --param_count;
        }
        // If there is no stopPackage at the end of the package, it searches the
        // next package.
        if (*it != UartPackage::stopPackage()) {
            continue;
        }
        current_package.setCommand(command);
        current_package.setData(params);
        result.emplace_back(current_package);
    }
    return result;
}

void PCBBridgeManager::incommingPackagesThread() {
    std::unique_lock<std::mutex> lock(mutex_);
    std::vector<uint8_t> incomming_data;
    do {
        if (serial_port_.IsDataAvailable()) {
            try{
                serial_port_.Read(incomming_data, 0 , static_cast<std::size_t>(check_interval_.count()));
            }
            catch(...) {}
            // Append the packages found into the current packages list.
            if(!incomming_data.empty()){
                packages_.splice(packages_.end(),decodePackage(incomming_data));
            }
            
        }
        else{
            cv_.wait_for(lock, check_interval_);
        }
    } while (!halt_);
}

}  // namespace noah_drivers
