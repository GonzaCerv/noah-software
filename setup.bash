#!/bin/bash
#This script downloads the source files and compiles ROS for ARMv6

#Configure Raspberry pi
echo Setting raspi-config settings
 #Expand filesystem
sudo raspi-config nonint do_expand_rootfs
 #Get boot cli
sudo raspi-config nonint do_boot_behaviour B1
 #Enable camera
sudo raspi-config nonint do_camera 1
 #Enable SSH
sudo raspi-config nonint do_ssh 1
 #Enable I2C
sudo raspi-config nonint do_i2c 1
 #Enable UART
sudo raspi-config nonint do_serial 1
 #GPU MEMORY
sudo raspi-config nonint do_memory_split 64

#Changing the swap space
echo Changing swap space size
swap_size='1024'
sudo sed -i "s/^CONF_SWAPSIZE*/CONF_SWAPSIZE=${swap_size}/" /etc/dphys-swapfile 

#Setting up ROS repositories
echo Adding ros repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

#Install requisites
echo Installing prerequisites
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y build-essential gdebi \
			python-pip \
			python-setuptools \
			git \
			checkinstall \
			cmake \
			libboost-system-dev \
			libboost-thread-dev \
			nano \
			tmux 
sudo pip install -U empy rosdep rosinstall_generator wstool rosinstall

#Initialize ROSDEP
echo Initializing rosdep
sudo rosdep init
rosdep update

#create ROS workspacee.
echo Create ros ws
mkdir Documents/ros_catkin_ws
cd Documents/ros_catkin_ws

#Fetch core packages.
echo Fetching ros packages
rosinstall_generator ros_comm diagnostics bond_core dynamic_reconfigure nodelet_core rosserial class_loader image_common vision_opencv image_transport_plugins pluginlib --rosdistro kinetic --deps --wet-only --exclude roslisp --tar > kinetic-robot-wet.rosinstall
wstool init -j8 src kinetic-robot-wet.rosinstall

#Install bridge-dev
echo Installing bridge-dev
mkdir ~/Documents/ros_catkin_ws/external_src
cd ~/Documents/ros_catkin_ws/external_src
git clone https://github.com/ros/console_bridge.git
cd console_bridge
cmake .
sudo checkinstall make install

# Resolve dependencies.
echo Resolving dependencies
cd ~/Documents/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:buster

# Building catkin workspace
echo Building catkin. (NOTE: IT TAKES A COUPLE OF HOURS)
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

# Source bashrc
echo Sourcing bashrc
echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
source /opt/ros/kinetic/setup.bash

# Finishing
echo Everything is OK. It is recommended to reboot before continue.


