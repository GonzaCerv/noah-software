/**
 * @file UartCommands.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief This class stores commmands received from UART.
 * @version 0.1
 * @date 2020-02-03
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

// C++ Standard libraries
#include <stdint.h>

#include <vector>

namespace noah_drivers {

// Enumeration that stores all the possible commands comming from UART.
enum class UartCommand {
    ECHO_CMD = 0x00,
    OFF = 0x02,
    LEFT_ENCODER_TICKS = 0x20,
    RIGHT_ENCODER_TICKS = 0x21,
    LEFT_SPEED = 0x40,
    RIGHT_SPEED = 0x41,
    EMPTY = 0xFE,
    ERROR = 0xFF,
};

class UartPackage {
   public:
    // Constructors
    UartPackage(const UartCommand& command = UartCommand::EMPTY,
                const std::vector<uint8_t>& parameters = std::vector<uint8_t>())
        : command_{command}, parameters_{parameters}  {}

    ~UartPackage(){};

    /// @brief Sets the command of the package.
    void setCommand(UartCommand command) { command_ = command; }

    /// @brief Converts the commands of UartCommand in the corresponding value in uint8_t.
    UartCommand getCommand() const { return command_; }

    /// @brief Sets the data of the package.
    void setData(std::vector<uint8_t> data) { parameters_ = data; }

    /// @brief Converts the commands of UartCommand in the corresponding value in uint8_t.
    std::vector<uint8_t> getData() const { return parameters_; }

    /// @brief Operator for accessing the arguments
    uint8_t& operator[](uint8_t idx) {
        if (idx > parameters_.size()) {
            throw std::overflow_error("Accessing element outside limits");
        }
        return parameters_[idx];
    }

    /// @brief Operator for accessing the arguments
    uint8_t operator[](uint8_t idx) const {
        if (idx > parameters_.size()) {
            throw std::overflow_error("Accessing element outside limits");
        }
        return parameters_[idx];
    }

    /// @brief Returns the start value for each package.
    static uint8_t startPackage() { return START_PACKAGE; };

    /// @brief Returns the stop value of the package.
    static uint8_t stopPackage() { return STOP_PACKAGE; }

    /// @brief Converts the commands of UartCommand in the corresponding value in uint8_t.
    static uint8_t commandToValue(UartCommand command) { return static_cast<uint8_t>(command); }

    /// @brief Converts the commands of UartCommand in the corresponding value in uint8_t.
    static UartCommand valueToCommand(uint8_t command) {
        switch (command) {
            case (static_cast<uint8_t>(UartCommand::ECHO_CMD)):
                return UartCommand::ECHO_CMD;
                break;
            case (static_cast<uint8_t>(UartCommand::OFF)):
                return UartCommand::OFF;
                break;
            case (static_cast<uint8_t>(UartCommand::LEFT_ENCODER_TICKS)):
                return UartCommand::LEFT_ENCODER_TICKS;
                break;
            case (static_cast<uint8_t>(UartCommand::RIGHT_ENCODER_TICKS)):
                return UartCommand::RIGHT_ENCODER_TICKS;
                break;
            case (static_cast<uint8_t>(UartCommand::LEFT_SPEED)):
                return UartCommand::LEFT_SPEED;
                break;
            case (static_cast<uint8_t>(UartCommand::RIGHT_SPEED)):
                return UartCommand::RIGHT_SPEED;
                break;
            default:
                return UartCommand::ERROR;
                break;
        }
    }

   private:
    /// @brief Command to be executed.
    UartCommand command_;

    /// @brief Parameters that comes with the command.
    std::vector<uint8_t> parameters_;

    /// @brief Start and stop package indicator.
    static const uint8_t START_PACKAGE = 0xA0;
    static const uint8_t STOP_PACKAGE = 0xB0;
};

}  // namespace noah_drivers