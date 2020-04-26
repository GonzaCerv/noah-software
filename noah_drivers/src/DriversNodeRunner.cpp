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

// C++ Standard Library
#include <cstdlib>
#include <exception>
#include <iostream>

// ROS
#include <ros/ros.h>

#include "ros/console.h"

// Bossa Nova
#include <noah_drivers/DriversNode.hpp>

using noah_drivers::DriversNode;

int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, "uart_comms_node");
        DriversNode pcb_comms_node;
        pcb_comms_node.run();
        return EXIT_SUCCESS;
    } catch (std::exception &e) {
        std::cerr << "Exception thrown!: " << e.what() << std::endl;
    }
    return EXIT_FAILURE;
}
