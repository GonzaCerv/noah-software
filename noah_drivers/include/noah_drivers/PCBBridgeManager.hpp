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
#include <unistd.h>

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
#include <flatbuffers/DataPackage_generated.h>
#include <flatbuffers/flatbuffers.h>
#include <serial/serial.h>


/// @brief default speed limits for each motor.
static constexpr const double DEFAULT_MIN_SPEED{0.1};
static constexpr const double DEFAULT_MAX_SPEED{0.6};
/// @brief queue size for topics.
static constexpr const uint32_t QUEUE_SIZE{100};
/// @brief size of the UART buffer that stores the incomming bytes.
static constexpr const uint32_t BUFFER_SIZE{100};
/// @brief Topics name.
static constexpr const char *TOPIC_LEFT_MOTOR_TICK_NAME{"motor_left_tick"};
static constexpr const char *TOPIC_RIGHT_MOTOR_TICK_NAME{"motor_right_tick"};
static constexpr const char *TOPIC_LEFT_MOTOR_SPEED_NAME{"motor_left_speed"};
static constexpr const char *TOPIC_RIGHT_MOTOR_SPEED_NAME{"motor_right_speed"};

namespace noah_drivers {

class PCBBridgeManager {
   public:
    PCBBridgeManager(ros::NodeHandle &nh, const std::string &port_name, const std::chrono::milliseconds &check_interval,
                     const double min_speed = DEFAULT_MIN_SPEED, const double max_speed = DEFAULT_MAX_SPEED);

    ~PCBBridgeManager();

   private:
    /// @brief Name of the package for logging.
    static constexpr const char *PACKAGE_NAME_{"PCB_PACKAGE_MANAGER"};

    /**
     * @brief This thread manages the communication with the PCB.
     *
     */
    void communicationThread();

    /**
     * @brief Callback for moving the robot.
     *
     * @param msg Information to send
     */
    void motorLeftSpeedCallback(const std_msgs::Float32 &msg);

    /**
     * @brief Callback for moving the robot.
     *
     * @param msg Information to send
     */
    void motorRightSpeedCallback(const std_msgs::Float32 &msg);

    /**
     * @brief Decode all the packages found in the last incomming UART communication. Publish all the information in
     * it's corresponding topic/service.
     *
     * @param new_data raw data that comes from USART.
     */
    void decodePackage(uint8_t *new_data);

    /// @brief Port used for communications.
    std::unique_ptr<serial::Serial> serial_port_;

    /// @brief Stores the thread that process each uart package.
    std::shared_ptr<std::thread> communication_thread_;

    /// @brief Manages the execution of the thread.
    std::condition_variable cv_;

    /// @brief Manages the accesing of the objects between thread.
    std::mutex mutex_;

    /// @brief Stops the execution of the thread.
    std::atomic_bool halt_;

    /// @brief Flags that indicates which information should be requested to PCB.
    std::atomic_bool request_target_speed_l_;
    std::atomic_bool request_target_speed_r_;
    std::atomic<float> target_speed_l_;
    std::atomic<float> target_speed_r_;

    /// @brief Nodehandler.
    ros::NodeHandle nh_;

    /// @brief Serialport name.
    std::string port_name_;

    std::chrono::milliseconds check_interval_;

    /// @brief Current duty of each motor.
    float current_duty_l_;
    float current_duty_r_;

    /// @brief Publisher for the left motor encoder tick.
    ros::Publisher left_motor_tick_pub_;

    /// @brief Publisher for the right motor encoder tick.
    ros::Publisher right_motor_tick_pub_;

    /// @brief Publisher for the left motor tick.
    ros::Subscriber left_motor_speed_sub_;

    /// @brief Publisher of the IR sensors.
    ros::Subscriber right_motor_speed_sub_;

    /// @brief Min-Max allowed duty for each motor..
    const double min_speed_;
    const double max_speed_;
};
}  // namespace noah_drivers