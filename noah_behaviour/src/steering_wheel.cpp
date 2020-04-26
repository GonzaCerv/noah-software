/**
 * @file DriversNode.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Provides communication between the hardware PCB and the raspberry.
 * @version 0.1
 * @date 2020-01-03
 *
 * @copyright Copyright (c) 2020
 */

// C++ Standard libraries
#include <linux/reboot.h>
#include <stdio.h>
#include <sys/reboot.h>
#include <unistd.h>

#include <memory>
#include <thread>

// ROS libraries
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"

// Noah libraries
#include "noah_drivers/DriversNode.hpp"

static constexpr const char *LEFT_MOTOR_TICK_NAME{"motor_left_tick"};
static constexpr const char *RIGHT_MOTOR_TICK_NAME{"motor_right_tick"};
static constexpr const char *LEFT_MOTOR_PWM_NAME{"motor_left_pwm"};
static constexpr const char *RIGHT_MOTOR_PWM_NAME{"motor_right_pwm"};
static constexpr const char *OFF_SERVICE_NAME{"smps_off"};

namespace noah_drivers {
DriversNode::DriversNode() : nh_("~") {
    // Start the manager for incommming packages.
    std::string port_name;
    nh_.getParam("/uart_comms_node/port_name", port_name);
    nh_.getParam("/uart_comms_node/min_duty", min_duty_);
    nh_.getParam("/uart_comms_node/max_duty", max_duty_);
    nh_.getParam("/uart_comms_node/update_rate", update_rate_);
    ROS_WARN_STREAM_NAMED("DriverNode", "update rate (Hz): " << std::to_string(update_rate_));
    ROS_WARN_STREAM_NAMED("DriverNode", "UART port name: " << port_name);

    // Convert Hz into ms
    auto update_interval_ms = (1.0f / static_cast<float>(update_rate_)) * 1000.0;

    pcb_bridge_manager_ =
        std::make_unique<PCBBridgeManager>(port_name, std::chrono::milliseconds(static_cast<int>(update_interval_ms)));

    current_duty_l_ = 0.0f;
    current_duty_r_ = 0.0f;

    // Publishers.
    left_motor_tick_pub_ = nh_.advertise<std_msgs::Int16>(LEFT_MOTOR_TICK_NAME, QUEUE_SIZE);
    right_motor_tick_pub_ = nh_.advertise<std_msgs::Int16>(RIGHT_MOTOR_TICK_NAME, QUEUE_SIZE);

    // Subscribers.
    left_motor_pwm_sub_ = nh_.subscribe(LEFT_MOTOR_PWM_NAME, QUEUE_SIZE, &DriversNode::motorLeftPWMCallback, this);
    right_motor_pwm_sub_ = nh_.subscribe(RIGHT_MOTOR_PWM_NAME, QUEUE_SIZE, &DriversNode::motorRightPWMCallback, this);

    // Services.
    off_srv_ = nh_.advertiseService(OFF_SERVICE_NAME, &DriversNode::offServiceCallback, this);
}

DriversNode::~DriversNode() {
    left_motor_tick_pub_.shutdown();
    right_motor_tick_pub_.shutdown();
}

void DriversNode::run() {
    // Rate of update of the node.
    ros::Rate rate(update_rate_);
    while (ros::ok()) {
        std::shared_ptr<UartPackage> incomming_package;
        do {
            incomming_package = pcb_bridge_manager_->check();
            if (incomming_package != nullptr) {
                executePackageCallaback(*incomming_package);
            }
        } while (incomming_package != nullptr);
        ros::spinOnce();
        rate.sleep();
    }
}

void DriversNode::executePackageCallaback(const UartPackage &current_package) {
    switch (current_package.getCommand()) {
        case UartCommand::ECHO_CMD:
            pcb_bridge_manager_->sendPackage(UartPackage(UartCommand::ECHO_CMD));
            break;
        case UartCommand::OFF:
            reboot(LINUX_REBOOT_CMD_POWER_OFF);
            break;
        case UartCommand::ENCODER_TICKS: {
            std_msgs::Int16 left_ticks;
            std_msgs::Int16 right_ticks;
            left_ticks.data = static_cast<int16_t>((current_package[0] << 8) | (current_package[1] & 0xff));
            right_ticks.data = static_cast<int16_t>((current_package[2] << 8) | (current_package[3] & 0xff));
            left_motor_tick_pub_.publish(left_ticks);
            right_motor_tick_pub_.publish(right_ticks);
            break;
        }
        default: {
            pcb_bridge_manager_->sendPackage(UartPackage(UartCommand::ERROR));
            break;
        }
    }
}

bool DriversNode::offServiceCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response) {
    ROS_WARN_STREAM_NAMED("DriverNode", "Turning off");
    response.success = true;
    response.message = "Turning off";
    system("poweroff");
    return true;
}

void DriversNode::motorLeftPWMCallback(const std_msgs::Float32 &msg) {
    if (min_duty_ > msg.data) {
        current_duty_l_ = min_duty_;
    } else if (max_duty_ < msg.data) {
        current_duty_l_ = max_duty_;
    } else {
        current_duty_l_ = msg.data;
    }
    std::vector<uint8_t> parameters;

    floatAsBytes.fval = current_duty_l_;
    parameters.emplace_back(floatAsBytes.bval[0]);
    parameters.emplace_back(floatAsBytes.bval[1]);
    parameters.emplace_back(floatAsBytes.bval[2]);
    parameters.emplace_back(floatAsBytes.bval[3]);
    floatAsBytes.fval = current_duty_r_;
    parameters.emplace_back(floatAsBytes.bval[0]);
    parameters.emplace_back(floatAsBytes.bval[1]);
    parameters.emplace_back(floatAsBytes.bval[2]);
    parameters.emplace_back(floatAsBytes.bval[3]);

    UartPackage package(UartCommand::PWM_DUTY, parameters);
    pcb_bridge_manager_->sendPackage(package);
}

void DriversNode::motorRightPWMCallback(const std_msgs::Float32 &msg) {
    if (min_duty_ > msg.data) {
        current_duty_r_ = min_duty_;
    } else if (max_duty_ < msg.data) {
        current_duty_r_ = max_duty_;
    } else {
        current_duty_r_ = msg.data;
    }
    std::vector<uint8_t> parameters;

    floatAsBytes.fval = current_duty_l_;
    parameters.emplace_back(floatAsBytes.bval[0]);
    parameters.emplace_back(floatAsBytes.bval[1]);
    parameters.emplace_back(floatAsBytes.bval[2]);
    parameters.emplace_back(floatAsBytes.bval[3]);
    floatAsBytes.fval = current_duty_r_;
    parameters.emplace_back(floatAsBytes.bval[0]);
    parameters.emplace_back(floatAsBytes.bval[1]);
    parameters.emplace_back(floatAsBytes.bval[2]);
    parameters.emplace_back(floatAsBytes.bval[3]);

    UartPackage package(UartCommand::PWM_DUTY, parameters);
    pcb_bridge_manager_->sendPackage(package);
}

}  // namespace noah_drivers