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

// Standard libraries
#include <memory>

// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// Noah libraries
#include "noah_drivers/PCBBridgeManager.hpp"

namespace noah_drivers {
class DriversNode {
   public:
    DriversNode();
    void chatterCallback(const std_msgs::String::ConstPtr& msg);
    int run();

   private:
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    ros::Publisher chatter_pub;
    std::unique_ptr<PCBBridgeManager> pcb_bridge_manager_;
};
}  // namespace noah_drivers
