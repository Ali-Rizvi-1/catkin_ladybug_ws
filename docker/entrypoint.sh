#!/bin/bash
set -e

# Start udev service
sudo service udev start

# Update user group membership without requiring logout
sudo groupadd -f flirimaging
sudo usermod -aG flirimaging ladybug_user
newgrp flirimaging

# Ensure USB and raw1394 devices are accessible
sudo mkdir -p /dev/bus/usb
sudo mkdir -p /dev/raw1394
sudo chmod 666 /dev/bus/usb
sudo chmod 666 /dev/raw1394

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Build catkin workspace if not already built
if [ ! -f ~/catkin_ws/devel/setup.bash ]; then
    cd ~/catkin_ws
    catkin_make
fi

source ~/catkin_ws/devel/setup.bash

# Execute the command passed to docker run
exec "$@"