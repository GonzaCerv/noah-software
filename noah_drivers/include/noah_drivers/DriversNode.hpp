/**
 * @file DriversNode.hpp
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
#include <memory>

// ROS libraries
#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_srvs/Trigger.h"

// Noah libraries
#include <noah_drivers/PCBBridgeManager.hpp>
#include <noah_drivers/UartPackage.hpp>

namespace noah_drivers {

class DriversNode {
   public:
    /**
     * @brief Construct a new DriversNode object
     */
    DriversNode();

    /**
     * @brief Destroy the DriversNode object
     */
    ~DriversNode();

    /*
     *@brief Executes the node.
     *
     * @return int status of the execution.
     */
    void run();

   private:
    /**
     * @brief Callback for when new package is available for processing.
     *
     * @param package_list commands to be processed.
     * @return true when process was successfull, false when not.
     */
    void executePackageCallaback(const UartPackage &package_list);

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

    /// @brief Node handler.
    ros::NodeHandle nh_;

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

    /// @brief Manager for incomming packages from UART.
    std::unique_ptr<PCBBridgeManager> pcb_bridge_manager_;

    /// @brief Current duty of each motor.
    float current_duty_l_;
    float current_duty_r_;

    /// @brief Min-Max allowed duty for each motor..
    double min_duty_;
    double max_duty_;

    int update_rate_;
};
}  // namespace noah_drivers