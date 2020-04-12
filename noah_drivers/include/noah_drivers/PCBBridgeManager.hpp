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
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <thread>

// ROS libraries
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_srvs/Trigger.h"

// Noah libraries
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

#include "noah_drivers/UartPackage.hpp"

namespace noah_drivers {

using LibSerial::AlreadyOpen;
using LibSerial::SerialPort;

class PCBBridgeManager {
   public:
    PCBBridgeManager(   ros::NodeHandle &nh,
                        const std::string &port_name, 
                        const std::chrono::milliseconds &check_interval_ = std::chrono::milliseconds(100));

    ~PCBBridgeManager();

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
     * @brief This thread checks if there is new packages to execute. 
     *        if there is a new package, executes the actions of the 
     *        command.
     *
     */
    void processPackagesThread();

    /**
     * @brief This thread manages the outgoing packages to the PCB.
     *
     */
    void outgoingPackagesThread();

    /**
     * @brief Thread that checks incomming data to UART.
     *
     */
    void incommingPackagesThread();

    /**
     * @brief Callback for the battery service.
     */
    bool offServiceCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response);

    /**
     * @brief Callback for moving the robot.
     */
    void motorLeftPWMCallback(const std_msgs::Float32 &msg);

    /**
     * @brief Callback for moving the robot.
     */
    void motorRightPWMCallback(const std_msgs::Float32 &msg);

    /**
     * @brief Decode all the packages found in the last incomming UART communication.
     *
     * @param new_data raw data that comes from USART.
     * @return new package to be processed. If there is no new package, it
     * returns a package with an EMPTY command.
     */
    std::list<UartPackage> decodePackage(const std::vector<uint8_t> &new_data);

    /// @brief queue size for topics.
    static const uint32_t QUEUE_SIZE = 1000;

    /**
     * @brief Union for stripping the float into  4 bytes parts.
     *
     */
    union {
        float fval;
        uint8_t bval[4];
    } floatAsBytes;

    /// @brief Port used for communications.
    SerialPort serial_port_;

    /// @brief Buffer of packages available to execute.
    std::list<UartPackage> incomming_packages_;

    /// @brief Buffer of packages available to execute.
    std::list<UartPackage> outgoing_packages_;

    /// @brief Stores the thread that process each uart package.
    std::shared_ptr<std::thread> process_packages_thread_;

    /// @brief Stores the thread that checks outgoing uart packages.
    std::shared_ptr<std::thread> outgoing_packages_thread_;

    /// @brief Stores the thread that checks incomming uart packages.
    std::shared_ptr<std::thread> incomming_packages_thread_;

    std::condition_variable cv_;

    /// @brief Manages the accesing of the objects between thread.
    std::mutex mutex_;

    /// @brief Stops the execution of the thread.
    std::atomic_bool halt_{false};

    /// @brief Nodehandler.
    ros::NodeHandle nh_;

    /// @brief Serialport name.
    std::string port_name_;

    std::chrono::milliseconds check_interval_;

    /// @brief Current duty of each motor.
    float current_duty_l_;
    float current_duty_r_;

    /// @brief Min-Max allowed duty for each motor..
    double min_speed_;
    double max_speed_;

    /// @brief Publisher for the left motor tick.
    ros::Publisher left_motor_tick_pub_;

    /// @brief Publisher of the IR sensors.
    ros::Publisher right_motor_tick_pub_;

    /// @brief Publisher for the left motor tick.
    ros::Subscriber left_motor_pwm_sub_;

    /// @brief Publisher of the IR sensors.
    ros::Subscriber right_motor_pwm_sub_;

    /// @brief Service for voltage checking.
    ros::ServiceServer off_srv_;
};
}  // namespace noah_drivers