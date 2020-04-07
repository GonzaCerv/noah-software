/**
 * @file PCBCommsNode.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Provides communication between the hardware PCB and the raspberry.
 * @version 0.1
 * @date 2020-01-03
 *
 * @copyright Copyright (c) 2020
 *  For exaplanations about the Uart library, check
 * https://raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
 */

// C++ Standard libraries
#include <stdio.h>
#include <memory>
#include <thread>

// ROS libraries
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"

// Noah libraries
#include "noah_drivers/PCBCommsNode.hpp"
#include "noah_msgs/IRFloor.h"
#include "noah_msgs/Pose.h"

static constexpr const char *BATTERY_PUB_NAME{"battery_voltage"};
static constexpr const char *BUTTON_PUB_NAME{"button"};
static constexpr const char *OFF_SERVICE_NAME{"off"};
static constexpr const char *LED_AQUIRE_PUB_NAME{"led_adquire"};
static constexpr const char *MOVE_SRV_NAME{"cmd_vel"};
static constexpr const char *IR_PUB_NAME{"ir_floor_sensor"};
static constexpr const char *POSE2D_PUB_NAME{"pose"};

namespace noah_drivers {
PCBCommsNode::PCBCommsNode() : nh_("~"), rate_(UPDATE_RATE) {
    // Start the manager for incommming packages.
    std::string port_name;
    nh_.getParam("/uart_comms_node/port_name", port_name);
    auto execute_package_callback = std::bind(&PCBCommsNode::executePackageCallaback, this, std::placeholders::_1);
    package_manager_ = std::make_unique<PackageManager>(port_name, execute_package_callback);
    package_manager_->start();

    // Publishers.
    battery_pub_ = nh_.advertise<std_msgs::Float32>(BATTERY_PUB_NAME, QUEUE_SIZE);
    button_pub_ = nh_.advertise<std_msgs::UInt8>(BUTTON_PUB_NAME, QUEUE_SIZE);
    ir_floor_pub_ = nh_.advertise<noah_msgs::IRFloor>(IR_PUB_NAME, QUEUE_SIZE);
    pose_pub_ = nh_.advertise<noah_msgs::Pose>(POSE2D_PUB_NAME, QUEUE_SIZE);

    // Subscribers.
    move_sub_ = nh_.subscribe(MOVE_SRV_NAME, QUEUE_SIZE, &PCBCommsNode::moveServiceCallback, this);

    // Services.
    off_srv_ = nh_.advertiseService(OFF_SERVICE_NAME, &PCBCommsNode::offServiceCallback, this);
    ir_back_floor_detected_ = true;
    ir_front_floor_detected_ = true;
}

PCBCommsNode::~PCBCommsNode() {
    battery_pub_.shutdown();
    button_pub_.shutdown();
    ir_floor_pub_.shutdown();
}

void PCBCommsNode::run() {
    while(ros::ok()){
        package_manager_->check();
        ros::spinOnce(); 
        rate_.sleep();
    }
}

void PCBCommsNode::executePackageCallaback(const UartPackage &command) {
    switch (command.command_) {
        case UartCommand::ECHO_CMD:
            package_manager_->sendPackage(UartPackage(UartCommand::ECHO_CMD));
            break;
        case UartCommand::OFF:
            system("poweroff");
            break;
        case UartCommand::BAT: {
            std_msgs::Float32 msg;
            floatAsBytes.bval[0] = command.parameters_[2];
            floatAsBytes.bval[1] = command.parameters_[3];
            floatAsBytes.bval[2] = command.parameters_[4];
            floatAsBytes.bval[3] = command.parameters_[5];
            msg.data = floatAsBytes.fval;
            battery_pub_.publish(msg);
            break;
        }
        case UartCommand::BUTTON_LONG_PRESS: {
            std_msgs::UInt8 msg;
            msg.data = LONG_PRESS;
            button_pub_.publish(msg);
            break;
        }
        case UartCommand::BUTTON_SHORT_PRESS: {
            std_msgs::UInt8 msg;
            msg.data = SHORT_PRESS;
            button_pub_.publish(msg);
            break;
        }
        case UartCommand::DETECT_IR_FRONT: {
            noah_msgs::IRFloor ir_floor;
            if (command.parameters_[2] == 1)
                ir_front_floor_detected_ = true;
            else
                ir_front_floor_detected_ = false;
            ir_floor.front_sensor = ir_front_floor_detected_;
            ir_floor.back_sensor = ir_back_floor_detected_;
            ir_floor_pub_.publish(ir_floor);
            break;
        }
        case UartCommand::DETECT_IR_BACK: {
            noah_msgs::IRFloor ir_floor;
            if (command.parameters_[2] == 1)
                ir_back_floor_detected_ = true;
            else
                ir_back_floor_detected_ = false;
            ir_floor.front_sensor = ir_front_floor_detected_;
            ir_floor.back_sensor = ir_back_floor_detected_;
            ir_floor_pub_.publish(ir_floor);
            break;
        }
        case UartCommand::POSE: {
            noah_msgs::Pose noah_pose;
            floatAsBytes.bval[0] = command.parameters_[2];
            floatAsBytes.bval[1] = command.parameters_[3];
            floatAsBytes.bval[2] = command.parameters_[4];
            floatAsBytes.bval[3] = command.parameters_[5];
            noah_pose.linear_velocity = static_cast<double>(floatAsBytes.fval);
            floatAsBytes.bval[0] = command.parameters_[6];
            floatAsBytes.bval[1] = command.parameters_[7];
            floatAsBytes.bval[2] = command.parameters_[8];
            floatAsBytes.bval[3] = command.parameters_[9];
            noah_pose.angular_velocity = static_cast<double>(floatAsBytes.fval);
            pose_pub_.publish(noah_pose);
        }
        default: {
            package_manager_->sendPackage(UartPackage(UartCommand::ERROR));
            break;
        }
    }
}

bool PCBCommsNode::offServiceCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response) {
    response.success = true;
    response.message = "Turning off";
    package_manager_->sendPackage(UartPackage(UartCommand::OFF));
    system("poweroff");
    return true;
}

void PCBCommsNode::moveServiceCallback(const geometry_msgs::Twist &msg) {
    floatAsBytes.fval = static_cast<float>(msg.linear.x);
    std::vector<uint8_t> parameters;
    floatAsBytes.fval = msg.linear.x;
    parameters.emplace_back(floatAsBytes.bval[0]);
    parameters.emplace_back(floatAsBytes.bval[1]);
    parameters.emplace_back(floatAsBytes.bval[2]);
    parameters.emplace_back(floatAsBytes.bval[3]);
    floatAsBytes.fval = msg.linear.z;
    parameters.emplace_back(floatAsBytes.bval[0]);
    parameters.emplace_back(floatAsBytes.bval[1]);
    parameters.emplace_back(floatAsBytes.bval[2]);
    parameters.emplace_back(floatAsBytes.bval[3]);
    UartPackage package(UartCommand::MOVE, parameters);
    package_manager_->sendPackage(package);
}

}  // namespace noah_drivers
