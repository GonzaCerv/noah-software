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
#include <linux/reboot.h>
#include <sys/reboot.h>
#include <unistd.h>

#include <memory>

// ROS libraries

// Noah libraries
#include "noah_drivers/PCBBridgeManager.hpp"

static constexpr const char *LEFT_MOTOR_TICK_NAME{"motor_left_tick"};
static constexpr const char *RIGHT_MOTOR_TICK_NAME{"motor_right_tick"};
static constexpr const char *LEFT_MOTOR_PWM_NAME{"motor_left_pwm"};
static constexpr const char *RIGHT_MOTOR_PWM_NAME{"motor_right_pwm"};
static constexpr const char *OFF_SERVICE_NAME{"smps_off"};

namespace noah_drivers {
PCBBridgeManager::PCBBridgeManager(ros::NodeHandle &nh, const std::string &port_name,
                                   const std::chrono::milliseconds &check_interval)
    : nh_{nh}, port_name_{port_name}, check_interval_{check_interval} {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    nh_.getParam("MIN_SPEED_METER_SEC", min_speed_);
    std::cerr << "min speed: " << std::to_string(min_speed_) << std::endl;
    nh_.getParam("MAX_SPEED_METER_SEC", max_speed_);
    ROS_WARN_STREAM_NAMED("DriverNode", "update interval (ms): " << std::to_string(check_interval_.count()));
    ROS_WARN_STREAM_NAMED("DriverNode", "UART port name: " << port_name_);

    // Stop the node if it isn't able to start the Uart connection.
    try {
        serial_port_.Open(port_name_);
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
    // Flush all the buffers.
    serial_port_.FlushInputBuffer();
    serial_port_.FlushOutputBuffer();

    // Publishers.
    left_motor_tick_pub_ = nh_.advertise<std_msgs::Int16>(LEFT_MOTOR_TICK_NAME, QUEUE_SIZE);
    right_motor_tick_pub_ = nh_.advertise<std_msgs::Int16>(RIGHT_MOTOR_TICK_NAME, QUEUE_SIZE);

    // Subscribers.
    left_motor_pwm_sub_ = nh_.subscribe(LEFT_MOTOR_PWM_NAME, QUEUE_SIZE, &PCBBridgeManager::motorLeftPWMCallback, this);
    right_motor_pwm_sub_ =
        nh_.subscribe(RIGHT_MOTOR_PWM_NAME, QUEUE_SIZE, &PCBBridgeManager::motorRightPWMCallback, this);

    // Services.
    off_srv_ = nh_.advertiseService(OFF_SERVICE_NAME, &PCBBridgeManager::offServiceCallback, this);

    // Threads.
    process_packages_thread_ = std::make_shared<std::thread>(std::bind(&PCBBridgeManager::processPackagesThread,
    this));
    incomming_packages_thread_ =
        std::make_shared<std::thread>(std::bind(&PCBBridgeManager::incommingPackagesThread, this));
    outgoing_packages_thread_ =
        std::make_shared<std::thread>(std::bind(&PCBBridgeManager::outgoingPackagesThread, this));

    // Start all the threads.
    process_packages_thread_->detach();
    incomming_packages_thread_->detach();
    outgoing_packages_thread_->detach();
}

PCBBridgeManager::~PCBBridgeManager() {
    std::cerr << "destructor called" << std::endl;
    left_motor_tick_pub_.shutdown();
    right_motor_tick_pub_.shutdown();
    halt_ = true;
    cv_.notify_all();
    std::cerr << "destroyerCalled" << std::endl;
    process_packages_thread_->join();
    incomming_packages_thread_->join();
    outgoing_packages_thread_->join();
    serial_port_.Close();
}

void PCBBridgeManager::processPackagesThread() {
    std::unique_lock<std::mutex> lock(mutex_);
    std::vector<uint8_t> incomming_data;
    while (!halt_) {
        if (!incomming_packages_.empty()) {
            auto current_package = incomming_packages_.front();

            switch (current_package.getCommand()) {
                case UartCommand::ECHO_CMD:
                    outgoing_packages_.emplace_back(UartPackage(UartCommand::ECHO_CMD));
                    break;
                case UartCommand::OFF:
                    reboot(LINUX_REBOOT_CMD_POWER_OFF);
                    break;
                case UartCommand::LEFT_ENCODER_TICKS: {
                    std_msgs::Int16 left_ticks;
                    left_ticks.data = static_cast<int16_t>((current_package[0] << 8) | (current_package[1] & 0xff));
                    left_motor_tick_pub_.publish(left_ticks);

                    // Create a new request for the next iteration.
                    std::vector<uint8_t> parameters(0x00, 0x00);
                    UartPackage package(UartCommand::LEFT_ENCODER_TICKS, parameters);
                    outgoing_packages_.emplace_back(package);
                    break;
                }
                case UartCommand::RIGHT_ENCODER_TICKS: {
                    std_msgs::Int16 right_ticks;
                    right_ticks.data = static_cast<int16_t>((current_package[0] << 8) | (current_package[1] & 0xff));
                    right_motor_tick_pub_.publish(right_ticks);

                    // Create a new request for the next iteration.
                    std::vector<uint8_t> parameters(0x00, 0x00);
                    UartPackage package(UartCommand::RIGHT_ENCODER_TICKS, parameters);
                    outgoing_packages_.emplace_back(package);
                    break;
                }
                default: {
                    outgoing_packages_.emplace_back(UartPackage(UartCommand::ERROR));
                    break;
                }
            }
            incomming_packages_.pop_front();
        }
        cv_.wait_for(lock, check_interval_);
    };
}

void PCBBridgeManager::outgoingPackagesThread() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!halt_) {
        if (!outgoing_packages_.empty()) {
            auto next_package = outgoing_packages_.front();
            sendPackage(next_package);
            outgoing_packages_.pop_front();
        }
        else{
            // If there is not data available to send, update 
            // the current status of the robot
            std::vector<uint8_t> parameters;
            parameters.emplace_back(0x00);
            parameters.emplace_back(0x00);
            UartPackage left_tick_package(UartCommand::LEFT_ENCODER_TICKS, parameters);
            outgoing_packages_.emplace_back(left_tick_package);
            UartPackage right_tick_package(UartCommand::RIGHT_ENCODER_TICKS, parameters);
            outgoing_packages_.emplace_back(right_tick_package);
        }
        cv_.wait_for(lock, check_interval_);
    }
}

void PCBBridgeManager::incommingPackagesThread() {
    std::unique_lock<std::mutex> lock(mutex_);
    std::vector<uint8_t> incomming_data;
    while (!halt_) {
        if (serial_port_.IsDataAvailable()) {
            try {
                serial_port_.Read(incomming_data, 0, static_cast<std::size_t>(check_interval_.count()));
            } catch (...) {
            }
            // Append the packages found into the current packages list.
            if (!incomming_data.empty()) {
                incomming_packages_.splice(incomming_packages_.end(), decodePackage(incomming_data));
            }
        } 
        cv_.wait_for(lock, check_interval_);
    }
}

bool PCBBridgeManager::offServiceCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response) {
    ROS_WARN_STREAM_NAMED("DriverNode", "Turning off");
    response.success = true;
    response.message = "Turning off";
    system("poweroff");
    return true;
}

void PCBBridgeManager::motorLeftPWMCallback(const std_msgs::Float32 &msg) {
    auto new_speed = -msg.data;
    if (min_speed_ > fabs(new_speed)) {
        current_duty_l_ = 0.0;
    } else if (max_speed_ <new_speed) {
        current_duty_l_ = max_speed_;
    } else if (max_speed_ < fabs(new_speed)) {
        current_duty_l_ = -max_speed_;
    } else {
        current_duty_l_ = new_speed;
    }
    std::vector<uint8_t> parameters;

    floatAsBytes.fval = current_duty_l_;
    parameters.emplace_back(floatAsBytes.bval[3]);
    parameters.emplace_back(floatAsBytes.bval[2]);
    parameters.emplace_back(floatAsBytes.bval[1]);
    parameters.emplace_back(floatAsBytes.bval[0]);

    UartPackage package(UartCommand::LEFT_SPEED, parameters);
    outgoing_packages_.emplace_back(package);
}

void PCBBridgeManager::motorRightPWMCallback(const std_msgs::Float32 &msg) {
    if (min_speed_ > fabs(msg.data)) {
        current_duty_r_ = 0.0;
    } else if (max_speed_ < msg.data) {
        current_duty_r_ = max_speed_;
    } else if (max_speed_ < fabs(msg.data)) {
        current_duty_r_ = -max_speed_;
    } else {
        current_duty_r_ = msg.data;
    }
    std::vector<uint8_t> parameters;

    floatAsBytes.fval = current_duty_r_;
    parameters.emplace_back(floatAsBytes.bval[3]);
    parameters.emplace_back(floatAsBytes.bval[2]);
    parameters.emplace_back(floatAsBytes.bval[1]);
    parameters.emplace_back(floatAsBytes.bval[0]);

    UartPackage package(UartCommand::RIGHT_SPEED, parameters);
    outgoing_packages_.emplace_back(package);
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

bool PCBBridgeManager::sendPackage(const UartPackage &package) {
    bool result = false;
    if (serial_port_.IsOpen()) {
        std::vector<uint8_t> buffer;
        buffer.emplace_back(UartPackage::startPackage());
        buffer.emplace_back(UartPackage::commandToValue(package.getCommand()));
        for (auto param : package.getData()) {
            buffer.emplace_back(param);
        }
        buffer.emplace_back(UartPackage::stopPackage());
        serial_port_.Write(buffer);
        result = true;
    }
    return result;
}

}  // namespace noah_drivers
