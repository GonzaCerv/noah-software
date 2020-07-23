/**
 * @file PCBBridgeManager.cpp
 * @author Gonzalo Cervetti (cervetti.g@gamil.com)
 * @brief Manages incomming packages from UART.
 * @version 0.1
 * @date 2020-02-09
 *
 * @copyright Copyright (c) 2020
 *
 */

// Standard library
#include <cmath>
#include <memory>

// ROS libraries

// Noah libraries
#include "noah_drivers/PCBBridgeManager.hpp"

using noah::ros::DataPackage;
using noah::ros::DataPackageBuilder;
using noah::ros::EncoderRequest;
using noah::ros::GetDataPackage;
using noah::ros::Side;

namespace noah_drivers {
PCBBridgeManager::PCBBridgeManager(ros::NodeHandle &nh, const std::string &port_name,
                                   const std::chrono::milliseconds &check_interval, const double min_speed,
                                   const double max_speed)
    : nh_{nh}, port_name_{port_name}, check_interval_{check_interval}, min_speed_{min_speed}, max_speed_{max_speed} {
    // Set log level to Debug.
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_WARN_STREAM_NAMED("DriverNode", "update interval (ms): " << std::to_string(check_interval_.count()));
    ROS_WARN_STREAM_NAMED("DriverNode", "UART port name: " << port_name_);

    serial_port_ = std::make_unique<serial::Serial>(port_name_, 115200,
                                                    serial::Timeout::simpleTimeout(check_interval_.count()));

    // Variables.
    halt_ = false;
    request_target_speed_l_ = false;
    request_target_speed_r_ = false;
    target_speed_l_ = 0.0f;
    target_speed_r_ = 0.0f;

    // Publishers.
    left_motor_tick_pub_ = nh_.advertise<std_msgs::Int16>(TOPIC_LEFT_MOTOR_TICK_NAME, QUEUE_SIZE);
    right_motor_tick_pub_ = nh_.advertise<std_msgs::Int16>(TOPIC_RIGHT_MOTOR_TICK_NAME, QUEUE_SIZE);

    // Subscribers.
    left_motor_speed_sub_ =
        nh_.subscribe(TOPIC_LEFT_MOTOR_SPEED_NAME, QUEUE_SIZE, &PCBBridgeManager::motorLeftSpeedCallback, this);
    right_motor_speed_sub_ =
        nh_.subscribe(TOPIC_RIGHT_MOTOR_SPEED_NAME, QUEUE_SIZE, &PCBBridgeManager::motorRightSpeedCallback, this);

    // Threads.
    communication_thread_ = std::make_shared<std::thread>(std::bind(&PCBBridgeManager::communicationThread, this));
    communication_thread_->detach();
}

PCBBridgeManager::~PCBBridgeManager() {
    // Shutdown all topics.
    left_motor_tick_pub_.shutdown();
    right_motor_tick_pub_.shutdown();

    // Notify threads to stop its execution.
    halt_ = true;
    cv_.notify_all();
    communication_thread_->join();

    // Close serial port.
    serial_port_->close();
}

void PCBBridgeManager::communicationThread() {
    std::unique_lock<std::mutex> lock(mutex_);

    while (!halt_) {
        // Checks if there is an incomming package to process
        if (serial_port_->available() != 0) {
            uint8_t incomming_data[BUFFER_SIZE];
            try {
                // Get all data available to read. Returns immediately
                serial_port_->read(&incomming_data[0], BUFFER_SIZE);
            } catch (...) {
                throw std::runtime_error("Error while trying to read from serial port.");
            }
            // Decode the package and publish the incomming information to its respective topic/service
            decodePackage(&incomming_data[0]);
        }

        flatbuffers::FlatBufferBuilder builder;
        // Force to send values even if they match with the default value.
        builder.ForceDefaults(true);
        DataPackageBuilder request_builder(builder);
        // Add the encoder to request information.
        // The encoder request is sent automatically on each iteration.
        auto encoder_response = EncoderRequest(Side::Side_All);
        request_builder.add_encoderRequest(&encoder_response);

        // Checks if there is a new request for left target speed.
        if (request_target_speed_l_) {
            request_target_speed_l_ = false;
            request_builder.add_targetSpeedLRequest(target_speed_l_);
        }

        // Checks if there is a new request for right target speed.
        if (request_target_speed_r_) {
            request_target_speed_r_ = false;
            request_builder.add_targetSpeedRRequest(target_speed_r_);
        }

        // Send the request
        auto request = request_builder.Finish();
        builder.Finish(request);
        uint8_t *request_buffer = builder.GetBufferPointer();
        uint16_t size = builder.GetSize();
        serial_port_->write(request_buffer, size);

        // Wait for the next iteration.
        cv_.wait_for(lock, check_interval_);
    }
}

void PCBBridgeManager::motorLeftSpeedCallback(const std_msgs::Float32 &msg) {
    target_speed_l_ = msg.data;

    // Checks the possible limits of the motors.
    if (max_speed_ < target_speed_l_) {
        ROS_INFO_STREAM_NAMED("PCBBridgeManager",
                              "Target speed for left motor is higher than allowed, setting max speed instead. ");
        target_speed_l_ = max_speed_;
    } else if ((min_speed_ > fabs(target_speed_l_)) && (target_speed_l_ != 0.0f)) {
        ROS_INFO_STREAM_NAMED("PCBBridgeManager",
                              "Target speed for left motor is lower than allowed, setting motor to 0. ");
        target_speed_l_ = 0.0f;
    } else if (max_speed_ < fabs(target_speed_l_)) {
        ROS_INFO_STREAM_NAMED("PCBBridgeManager",
                              "Target speed for left motor is higher than allowed, setting max speed instead. ");
        target_speed_l_ = -max_speed_;
    }

    request_target_speed_l_ = true;
}

void PCBBridgeManager::motorRightSpeedCallback(const std_msgs::Float32 &msg) {
    target_speed_r_ = msg.data;

    // Checks the possible limits of the motors.
    if (max_speed_ < target_speed_r_) {
        ROS_INFO_STREAM_NAMED("PCBBridgeManager",
                              "Target speed for left motor is higher than allowed, setting max speed instead. ");
        target_speed_r_ = max_speed_;
    } else if ((min_speed_ > fabs(target_speed_r_)) && (target_speed_r_ != 0.0f)) {
        ROS_INFO_STREAM_NAMED("PCBBridgeManager",
                              "Target speed for left motor is lower than allowed, setting motor to 0. ");
        target_speed_r_ = 0.0f;
    } else if (max_speed_ < fabs(target_speed_r_)) {
        ROS_INFO_STREAM_NAMED("PCBBridgeManager",
                              "Target speed for left motor is higher than allowed, setting max speed instead. ");
        target_speed_r_ = -max_speed_;
    }

    request_target_speed_r_ = true;
}

void PCBBridgeManager::decodePackage(uint8_t *new_data) {
    flatbuffers::FlatBufferBuilder builder;

    // Deserialize the incoming package.
    auto incomming_package = GetDataPackage(new_data);

    // Process response for encoders.
    if (flatbuffers::IsFieldPresent(incomming_package, DataPackage::VT_ENCODERRESPONSE)) {
        auto encoder_response = incomming_package->encoderResponse();
        // Publish in ROS the ticks
        std_msgs::Int16 ticks_l;
        ticks_l.data = encoder_response->ticks_l();
        left_motor_tick_pub_.publish(ticks_l);

        std_msgs::Int16 ticks_r;
        ticks_r.data = encoder_response->ticks_r();
        right_motor_tick_pub_.publish(ticks_r);
    }
}

}  // namespace noah_drivers
