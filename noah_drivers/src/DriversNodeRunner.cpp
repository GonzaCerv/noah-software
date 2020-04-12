/**
 * @file UARTBridgeNodeRunner.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Runner for the UART Bridge Node.
 * @version 0.1
 * @date 2020-01-03
 *
 * @copyright Copyright (c) 2020
 *
 */

// Standard libraries
#include <iostream>

// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// Noah libraries
#include <noah_drivers/DriversNode.hpp>

using noah_drivers::DriversNode;

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "noah_drivers");
    DriversNode driver_node;
    return driver_node.run();
  } catch (std::exception& e) {
    std::cerr << "Exception thrown!: " << e.what() << std::endl;
  }
  return EXIT_FAILURE;
}
