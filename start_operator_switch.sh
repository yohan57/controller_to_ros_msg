#!/bin/bash

# This script sets up the ROS 2 environment and runs the operator_switch_node.

# Source ROS 2 galactic environment to make rclpy and other ROS packages available
if [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
else
    echo "Error: ROS Galactic setup file not found." >&2
    exit 1
fi

# Source the workspace overlay
if [ -f "/home/linkxavier/ros2_ws/install/setup.bash" ]; then
    source /home/linkxavier/ros2_ws/install/setup.bash
else
    echo "Warning: Workspace setup file not found." >&2
fi

# Run the operator switch node directly using Python
echo "Starting operator_switch_node..."
python3 /home/linkxavier/ros2_ws/src/controller_to_ros_msg/src/operator_switch_node.py
