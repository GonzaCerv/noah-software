/**
 * @file Uart.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Provides communication between the hardware PCB and the raspberry.
 * @version 0.1
 * @date 2020-01-03
 *
 * @copyright Copyright (c) 2020
 *  For exaplanations about the Uart library, check
 * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
 */

// C++ Standard libraries
#include <errno.h>  // Error integer and strerror() function
#include <fcntl.h>  //Used for Uart
#include <stdio.h>
#include <termios.h>  //Used for  Uart
#include <unistd.h>   //Used for Uart

// Noah libraries
#include "noah_drivers/Uart.hpp"

namespace noah_drivers {
Uart::Uart(const std::string &serial_port) : serial_port_name_(serial_port.c_str()) { is_port_open_ = false; }

Uart::~Uart() {
    if (is_port_open_) closePort();
}

bool Uart::openPort() {
    if (!is_port_open_) {
        serial_port_ = open(serial_port_name_, O_RDWR);
        if (serial_port_ < 0) {
            ROS_ERROR_STREAM_NAMED(PACKAGE_NAME, "Unable to open serial port: " << errno << strerror(errno));
            return false;
        }
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        // Read in existing settings, and handle any error
        if (tcgetattr(serial_port_, &tty) != 0) {
            ROS_ERROR_STREAM_NAMED(PACKAGE_NAME, "Error reading settings: " << errno << strerror(errno));
            return false;
        }
        // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~PARENB;
        // Clear stop field, only one stop bit used in communication
        tty.c_cflag &= ~CSTOPB;
        // 8 bits per byte.
        tty.c_cflag |= CS8;
        // Disable RTS/CTS hardware flow control
        tty.c_cflag &= ~CRTSCTS;
        // Turn on READ & ignore ctrl lines (CLOCAL = 1)
        tty.c_cflag |= CREAD | CLOCAL;
        // Disable canonincal mode.
        tty.c_lflag &= ~ICANON;
        // Disable echo
        tty.c_lflag &= ~ECHO;
        // Disable erasure
        tty.c_lflag &= ~ECHOE;
        // Disable new-line echo
        tty.c_lflag &= ~ECHONL;
        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_lflag &= ~ISIG;
        // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        // Disable any special handling of received bytes
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~OPOST;
        // Prevent conversion of newline to carriage return/line feed
        tty.c_oflag &= ~ONLCR;
        // Wait for up to 0.1s (1 deciseconds), returning as soon as any data is received.
        tty.c_cc[VTIME] = 1;
        tty.c_cc[VMIN] = 0;
        // Set baudrate.
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);
        // Save tty settings, also checking for error
        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            ROS_ERROR_STREAM_NAMED(PACKAGE_NAME, "Error setting port settings: " << errno << strerror(errno));
        }
        is_port_open_ = true;
    }
    return true;
}

bool Uart::closePort() {
    if (is_port_open_) {
        is_port_open_ = false;
        return !close(serial_port_);
    }
    return true;
}

bool Uart::transmit(const std::vector<uint8_t> &data) {
    if (is_port_open_) {
        write(serial_port_, &data[0], data.size());
        return true;
    }
    return false;
}

std::vector<uint8_t> Uart::receive(uint32_t size) {
    std::vector<uint8_t> result;
    if (is_port_open_) {
        // Allocate memory for read buffer, set size according to your needs
        char read_buf[size];
        memset(&read_buf, '\0', size);

        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        int num_bytes = read(serial_port_, &read_buf, sizeof(read_buf));

        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
        if (num_bytes < 0) {
            ROS_ERROR_STREAM_NAMED(PACKAGE_NAME, "Error while trying to read " << errno << strerror(errno));
            return result;
        }
        for (auto i = 0; i < num_bytes; ++i) {
            result.push_back(static_cast<uint8_t>(read_buf[i]));
        }
    }
    return result;
}

bool Uart::isOpen() { return is_port_open_; }

}  // namespace noah_drivers
