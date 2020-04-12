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

// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// Noah libraries
#include "noah_drivers/DriversNode.hpp"

namespace noah_drivers {
    DriversNode::DriversNode() {
      pcb_bridge_manager_ = std::make_unique<PCBBridgeManager>(nh_, "/dev/ttyACM0", std::chrono::milliseconds(10));
    }

    int DriversNode::run(){
      ros::spin();
      return 0;
    }
}  // namespace noah_drivers
