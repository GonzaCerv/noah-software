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
    OFF = 0x01,
    LED_AQUIRE = 0x02,
    LED_RED_ON = 0x03,
    LED_RED_OFF = 0x04,
    LED_GREEN_ON = 0x05,
    LED_GREEN_OFF = 0x06,
    LED_RELEASE = 0x07,
    BUTTON_LONG_PRESS = 0x08,
    BUTTON_SHORT_PRESS = 0x09,
    DETECT_IR_FRONT = 0x0A,
    DETECT_IR_BACK = 0x0B,
    SERVO_MOVE = 0x10,
    BAT = 0x40,
    MOTOR_KP = 0x41,
    MOTOR_KI = 0x42,
    MOTOR_KD = 0x43,
    MOVE = 0x80,
    POSE = 0x81,
    EMPTY = 0xFE,
    ERROR = 0xFF,
};

class UartPackage {
   public:
    // Constructors
    UartPackage(const UartCommand& command = UartCommand::EMPTY,
                std::vector<uint8_t> parameters = std::vector<uint8_t>())
        : command_(command), parameters_(parameters) {}

    ~UartPackage(){};

    /// @brief Converts the commands of UartCommand in the corresponding value in uint8_t.
    uint8_t getCommandValue() { return static_cast<uint8_t>(command_); }

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
            case (static_cast<uint8_t>(UartCommand::LED_AQUIRE)):
                return UartCommand::LED_AQUIRE;
                break;
            case (static_cast<uint8_t>(UartCommand::LED_RED_ON)):
                return UartCommand::LED_RED_ON;
                break;
            case (static_cast<uint8_t>(UartCommand::LED_RED_OFF)):
                return UartCommand::LED_RED_OFF;
                break;
            case (static_cast<uint8_t>(UartCommand::LED_GREEN_ON)):
                return UartCommand::LED_GREEN_ON;
                break;
            case (static_cast<uint8_t>(UartCommand::LED_GREEN_OFF)):
                return UartCommand::LED_GREEN_OFF;
                break;
            case (static_cast<uint8_t>(UartCommand::LED_RELEASE)):
                return UartCommand::LED_RELEASE;
                break;
            case (static_cast<uint8_t>(UartCommand::BUTTON_LONG_PRESS)):
                return UartCommand::BUTTON_LONG_PRESS;
                break;
            case (static_cast<uint8_t>(UartCommand::BUTTON_SHORT_PRESS)):
                return UartCommand::BUTTON_SHORT_PRESS;
                break;
            case (static_cast<uint8_t>(UartCommand::SERVO_MOVE)):
                return UartCommand::SERVO_MOVE;
                break;
            case (static_cast<uint8_t>(UartCommand::DETECT_IR_BACK)):
                return UartCommand::DETECT_IR_BACK;
                break;
            case (static_cast<uint8_t>(UartCommand::BAT)):
                return UartCommand::BAT;
                break;
            case (static_cast<uint8_t>(UartCommand::MOTOR_KP)):
                return UartCommand::MOTOR_KP;
                break;
            case (static_cast<uint8_t>(UartCommand::MOTOR_KI)):
                return UartCommand::MOTOR_KI;
                break;
            case (static_cast<uint8_t>(UartCommand::MOTOR_KD)):
                return UartCommand::MOTOR_KD;
                break;
            case (static_cast<uint8_t>(UartCommand::MOVE)):
                return UartCommand::MOVE;
                break;
            case (static_cast<uint8_t>(UartCommand::POSE)):
                return UartCommand::POSE;
                break;
            default:
                return UartCommand::ERROR;
                break;
        }
    }

    /// @brief Command to be executed.
    UartCommand command_;

    /// @brief Parameters that comes with the command.
    std::vector<uint8_t> parameters_;

    private:
    /// @brief Start and stop package indicator.
    static const uint8_t START_PACKAGE = 0xA0;
    static const uint8_t STOP_PACKAGE = 0xB0;
};

}  // namespace noah_drivers