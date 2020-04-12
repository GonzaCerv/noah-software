#!/usr/bin/env bash

# Author:        Gonzalo Cervetti (cervetti.g@gmail.com)
#
# Description:   This script is called when the roslaunch starts. This sets
#                all the environment variables for the project

# PCB communication
export PCB_UART_PORT=/dev/ttyS0
export PCB_UART_UPDATE_RATE_HZ=20

# Physical characteristics of the robot
export TICK_METERS=3653
export BASE_WIDTH=0.2

# Motor Drive configuration
export PID_MIN=10.0
export PID_MAX=700.0
export PID_P=50.0
export PID_I=200.0
export PID_D=2.0