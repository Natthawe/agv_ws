#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=102
ros2 launch launch/moteus.launch.py
