#!/bin/bash

# This script activates the CAN interface and starts the Piper single controller node.

# --- Configuration ---
CAN_INTERFACE="can0"
BITRATE="1000000"
# Assuming the can_activate.sh script is in a fixed location
CAN_ACTIVATE_SCRIPT="/home/linkxavier/piper_sdk/piper_sdk/can_activate.sh"

# --- Source ROS Environment ---
echo "Sourcing ROS 2 environment..."

# Check if ROS setup files exist before sourcing
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Warning: ROS Humble setup file not found."
fi

if [ -f "/home/linkxavier/ros2_ws/install/setup.bash" ]; then
    source /home/linkxavier/ros2_ws/install/setup.bash
else
    echo "Warning: Workspace setup file not found."
fi

# --- Main Script ---

echo "### Step 1: Activating CAN interface '$CAN_INTERFACE' ###"

if [ ! -f "$CAN_ACTIVATE_SCRIPT" ]; then
    echo "Error: CAN activation script not found at $CAN_ACTIVATE_SCRIPT"
    exit 1
fi

# Execute the activation script
bash "$CAN_ACTIVATE_SCRIPT" "$CAN_INTERFACE" "$BITRATE"
if [ $? -ne 0 ]; then
    echo "Error: CAN activation script failed."
    exit 1
fi

# Verify that the interface is up
if ! ip link show $CAN_INTERFACE | grep -q "state UP"; then
    echo "Error: Failed to bring up CAN interface '$CAN_INTERFACE'."
    exit 1
fi
echo "CAN interface '$CAN_INTERFACE' is UP."


echo -e "\n### Step 2: Launching Piper Controller ###"
ros2 run piper piper_single_ctrl --ros-args -p can_port:=$CAN_INTERFACE -p auto_enable:=true -p gripper_exist:=true -p gripper_multiplier:=1.0

echo "Script finished."
