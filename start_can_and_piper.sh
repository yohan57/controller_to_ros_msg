#!/bin/bash

# This script activates the CAN interface and starts the Piper single controller node.

# --- Configuration ---
CAN_INTERFACE="can0"
BITRATE="1000000"
# Assuming the can_activate.sh script is in a fixed location
CAN_ACTIVATE_SCRIPT="/home/linkxavier/scripts/can_activate.sh"

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

echo "Script finished."
