/**
 * @file PCBCommsNode.hpp
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
#include "std_srvs/Trigger.h"

// Noah libraries
#include <noah_msgs/Move.h>
#include <noah_drivers/Uart.hpp>
#include <noah_drivers/UartPackage.hpp>
#include <noah_drivers/PackageManager.hpp>

namespace noah_drivers {

class PCBCommsNode{
   public:
    /**
     * @brief Construct a new PCBCommsNode object
     */
    PCBCommsNode();

    /**
     * @brief Destroy the PCBCommsNode object
     */
    ~PCBCommsNode();

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
     * @param command command to be processed.
     * @return true when process was successfull, false when not.
     */
    void executePackageCallaback(const UartPackage &command);

    /**
     * @brief Callback for the battery service.
     */
    bool offServiceCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response);

    /**
     * @brief Callback for moving the robot.
     */
    void moveServiceCallback(const geometry_msgs::Twist &msg);

    /// @brief queue size for topics.
    static const uint32_t QUEUE_SIZE = 5;

    /// @brief message for long press event.
    static const uint8_t LONG_PRESS = 1;

    /// @brief message for short press event.
    static const uint8_t SHORT_PRESS = 1;

    /**
     * @brief Name of the package for logging.
     */
    static constexpr const char *PACKAGE_NAME{"PCB_COMMS_NODE"};

    /**
     * @brief Update rate of the node [hz].
     */
    static constexpr const double UPDATE_RATE{10};

    /**
     * @brief Union for stripping the float into  4 bytes parts.
     *
     */
    union {
        float fval;
        uint8_t bval[4];
    } floatAsBytes;

    typedef enum {
        REQUESTED,
        STOPPED,
    } requestStatus;

    /// @brief Node handler.
    ros::NodeHandle nh_;

    /// @brief Rate of update of the node.
    ros::Rate rate_;

    /// @brief Publisher of the IR sensors.
    ros::Publisher battery_pub_;

    /// @brief  Publisher of the button.
    ros::Publisher button_pub_;

    /// @brief Publisher of the IR sensors.
    ros::Publisher ir_floor_pub_;

    /// @brief Publisher of the robots current speed.
    ros::Publisher pose_pub_;

    /// @brief Service for voltage checking.
    ros::Subscriber move_sub_;

    /// @brief Service for voltage checking.
    ros::ServiceServer off_srv_;

    /// @brief publisher for the battery.
    float battery_voltage_pub_;

    /// @brief stores the status of the sensors.
    bool ir_back_floor_detected_;

    /// @brief stores the status of the sensors.
    bool ir_front_floor_detected_;

    /// @brief Manager for incomming packages from UART.
    std::unique_ptr<PackageManager> package_manager_;
};
}  // namespace noah_drivers