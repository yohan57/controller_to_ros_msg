#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/linkxavier/ros2_ws/install/setup.bash

# Run the operator switch node
ros2 run controller_to_ros_msg operator_switch_node
