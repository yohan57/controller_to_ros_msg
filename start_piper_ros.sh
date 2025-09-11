#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/linkxavier/ros2_ws/install/setup.bash

wait

ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=true -p gripper_exist:=true -p gripper_multiplier:=1.0


