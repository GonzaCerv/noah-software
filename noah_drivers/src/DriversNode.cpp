/**
 * @file DriversNode.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Provides all the drivers necessary for the robot to work
 * @version 0.1
 * @date 2020-01-03
 *
 * @copyright Copyright (c) 2020
 *
 */

// Standard libraries
#include <chrono>
#include <stdexcept>

// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// Noah libraries
#include "noah_drivers/DriversNode.hpp"

namespace noah_drivers {
DriversNode::DriversNode() {
    std::string pcb_uart_port;
    if (!nh_.getParam("PCB_BRIDGE_UART_PORT", pcb_uart_port)) {
        throw std::runtime_error("The parameter PCB_BRIDGE_UART_PORT is not set.");
    }

    int32_t bridge_update_rate_ms;
    if (!nh_.getParam("PCB_BRIDGE_CHECK_INTERVAL_MS", bridge_update_rate_ms)) {
        throw std::runtime_error("The parameter PCB_BRIDGE_CHECK_INTERVAL_MS is not set.");
    }

    double motor_min_speed;
    if (!nh_.getParam("MOTOR_MIN_SPEED_METER_SEC", motor_min_speed)) {
        throw std::runtime_error("The parameter MOTOR_MIN_SPEED_METER_SEC is not set.");
    }

    double motor_max_speed;
    if (!nh_.getParam("MOTOR_MAX_SPEED_METER_SEC", motor_max_speed)) {
        throw std::runtime_error("The parameter MOTOR_MIN_SPEED_METER_SEC is not set.");
    }

    pcb_bridge_manager_ = std::make_unique<PCBBridgeManager>(
        nh_, pcb_uart_port, std::chrono::milliseconds(bridge_update_rate_ms), motor_min_speed, motor_max_speed);
}

int DriversNode::run() {
    ros::spin();
    return 0;
}
}  // namespace noah_drivers
