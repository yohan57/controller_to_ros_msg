#!/bin/bash

# This script performs the following actions:
# 1. Activates the CAN interface.
# 2. Launches the Piper robot ROS 2 nodes.
# 3. Launches the Xbox controller ROS 2 node.
# 4. Launches a GStreamer camera stream.
# 5. Monitors the CAN interface for data.
# It must be run with sudo privileges.

# --- Configuration ---
CAN_INTERFACE="can0"
BITRATE="1000000"
# Use eval to handle the tilde (~) expansion for the home directory
CAN_ACTIVATE_SCRIPT=$(eval echo "/home/linkxavier/piper_sdk/piper_sdk/can_activate.sh")
MONITOR_TIMEOUT=5
GST_DEVICE1="/dev/video4"
GST_DEVICE2="/dev/video10"

# --- Pre-flight Checks ---
if ! command -v candump &> /dev/null; then
    echo "Error: 'candump' command not found. Please install can-utils."
    exit 1
fi

if [ ! -f "$CAN_ACTIVATE_SCRIPT" ]; then
    echo "Error: CAN activation script not found at $CAN_ACTIVATE_SCRIPT"
    exit 1
fi

# --- Cleanup Function ---
cleanup() {
    echo -e "\n\nCaught Signal... Shutting down all processes."
    # Kill all processes in the process group of this script.
    if [ -n "$CANDUMP_PID" ]; then
        kill $CANDUMP_PID 2>/dev/null
    fi
    kill -s SIGINT 0
    wait
    echo "All processes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

# --- Main Script ---

echo "### Step 1: Activating CAN interface '$CAN_INTERFACE' ###"
# Execute the activation script
bash "$CAN_ACTIVATE_SCRIPT" "$CAN_INTERFACE" "$BITRATE"
if [ $? -ne 0 ]; then
    echo "Error: CAN activation script failed."
    exit 1
fi

# Verify that the interface is up and running
if ! ip link show $CAN_INTERFACE | grep -q "state UP"; then
    echo "Error: Failed to bring up CAN interface '$CAN_INTERFACE' after activation script."
    exit 1
fi
echo "CAN interface is UP."

echo -e "\n### Step 2: Performing initial check for CAN data... ###"

# Use timeout to run candump. If it receives at least one frame (-n 1), it succeeds.
if timeout ${MONITOR_TIMEOUT}s candump -n 1 ${CAN_INTERFACE} | read; then
    echo "Success: Initial data received on '$CAN_INTERFACE'."
else
    echo "Error: No CAN data received on '$CAN_INTERFACE' during the initial check."
    echo "Please check your hardware connections and configuration."
    exit 1
fi

echo -e "\n### Step 3: Launching ROS nodes and GStreamer ###"
echo "NOTE: Make sure you have sourced your ROS 2 workspace in this terminal."

# Start the piper robot launch file in the background
echo "Launching Piper robot..."
ros2 launch piper start_single_piper.launch.py &

# Start the controller node in the background
echo "Launching controller node..."
python3 /home/linkxavier/ros2_ws/src/controller_to_ros_msg/src/xbox_to_pos_node.py &

# Start the camera stream in the background
echo "Launching camera stream..."
gst-launch-1.0 v4l2src device=${GST_DEVICE1} ! video=x-raw, width=1280, height=720 ! autovideosink &
gst-launch-1.0 v4l2src device=${GST_DEVICE2} ! video=x-raw, width=1280, height=720 ! autovideosink &

echo "All components launched."

echo -e "\n### Step 4: Starting continuous CAN monitoring. Press Ctrl+C to stop. ###"

# Variable to store the timestamp of the last received message
LAST_DATA_TIMESTAMP=$(date +%s)

# Run candump in the background, piping its output to a while loop
candump ${CAN_INTERFACE} | while read -r line; do
    if [ -n "$line" ]; then
        # If a line is read, update the timestamp
        LAST_DATA_TIMESTAMP=$(date +%s)
        # Provide continuous feedback that things are working
        #!/bin/bash

# This script performs the following actions:
# 1. Activates the CAN interface.
# 2. Launches the Piper robot ROS 2 nodes.
# 3. Launches the Xbox controller ROS 2 node.
# 4. Launches a GStreamer camera stream.
# 5. Monitors the CAN interface for data.
# It must be run with sudo privileges.

# --- Configuration ---
CAN_INTERFACE="can0"
BITRATE="1000000"
# Use eval to handle the tilde (~) expansion for the home directory
CAN_ACTIVATE_SCRIPT=$(eval echo "/home/linkxavier/piper_sdk/piper_sdk/can_activate.sh")
MONITOR_TIMEOUT=5
GST_DEVICE1="/dev/video4"
GST_DEVICE2="/dev/video10"

# --- Process Management ---
ROS_LAUNCH_PID=""
CONTROLLER_PID=""
GST_PID1=""
GST_PID2=""
CANDUMP_PID=""
MONITOR_PID=""

# --- Pre-flight Checks ---
if ! command -v candump &> /dev/null; then
    echo "Error: 'candump' command not found. Please install can-utils."
    exit 1
fi

if [ ! -f "$CAN_ACTIVATE_SCRIPT" ]; then
    echo "Error: CAN activation script not found at $CAN_ACTIVATE_SCRIPT"
    exit 1
fi

# --- Cleanup Function ---
cleanup() {
    echo -e "

Caught Signal... Shutting down all processes."
    
    # Kill processes in reverse order of creation.
    if [ -n "$MONITOR_PID" ]; then kill $MONITOR_PID 2>/dev/null; fi
    if [ -n "$CANDUMP_PID" ]; then kill $CANDUMP_PID 2>/dev/null; fi
    if [ -n "$GST_PID2" ]; then kill $GST_PID2 2>/dev/null; fi
    if [ -n "$GST_PID1" ]; then kill $GST_PID1 2>/dev/null; fi
    
    # For ROS nodes, send SIGINT for a graceful shutdown.
    if [ -n "$CONTROLLER_PID" ]; then 
        echo "Stopping controller node (PID: $CONTROLLER_PID)..."
        kill -SIGINT $CONTROLLER_PID 2>/dev/null
    fi
    if [ -n "$ROS_LAUNCH_PID" ]; then
        echo "Stopping ROS launch (PID: $ROS_LAUNCH_PID)..."
        # ros2 launch needs SIGINT to shut down nodes gracefully.
        kill -SIGINT $ROS_LAUNCH_PID 2>/dev/null
        # Give it a moment to propagate the signal to its children.
        sleep 1
        # As a fallback, kill any child processes of the launch script that might have been left behind.
        pkill -P $ROS_LAUNCH_PID
    fi
    
    wait # Wait for child processes to terminate.
    echo "All processes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

# --- Main Script ---

echo "### Step 1: Activating CAN interface '$CAN_INTERFACE' ###"
# Execute the activation script
bash "$CAN_ACTIVATE_SCRIPT" "$CAN_INTERFACE" "$BITRATE"
if [ $? -ne 0 ]; then
    echo "Error: CAN activation script failed."
    exit 1
fi

# Verify that the interface is up and running
if ! ip link show $CAN_INTERFACE | grep -q "state UP"; then
    echo "Error: Failed to bring up CAN interface '$CAN_INTERFACE' after activation script."
    exit 1
fi
echo "CAN interface is UP."

echo -e "
### Step 2: Performing initial check for CAN data... ###"

# Use timeout to run candump. If it receives at least one frame (-n 1), it succeeds.
if timeout ${MONITOR_TIMEOUT}s candump -n 1 ${CAN_INTERFACE} | read; then
    echo "Success: Initial data received on '$CAN_INTERFACE'."
else
    echo "Error: No CAN data received on '$CAN_INTERFACE' during the initial check."
    echo "Please check your hardware connections and configuration."
    exit 1
fi

echo -e "
### Step 3: Launching ROS nodes and GStreamer ###"
echo "NOTE: Make sure you have sourced your ROS 2 workspace in this terminal."

# Start the piper robot launch file in the background
echo "Launching Piper robot..."
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=true -p gripper_exist:=true -p gripper_val_mutiple:=1.0 &
ROS_LAUNCH_PID=$!

# Start the controller node in the background
echo "Launching controller node..."
python3 /home/linkxavier/ros2_ws/src/controller_to_ros_msg/src/xbox_to_pos_node.py &
CONTROLLER_PID=$!

# Start the camera stream in the background
echo "Launching camera stream..."
gst-launch-1.0 v4l2src device=${GST_DEVICE1} ! video=x-raw, width=1280, height=720 ! autovideosink &
GST_PID1=$!
gst-launch-1.0 v4l2src device=${GST_DEVICE2} ! video=x-raw, width=1280, height=720 ! autovideosink &
GST_PID2=$!

echo "All components launched."

echo -e "
### Step 4: Starting continuous CAN monitoring. Press Ctrl+C to stop. ###"

# Variable to store the timestamp of the last received message
LAST_DATA_TIMESTAMP=$(date +%s)

# Run candump in the background, piping its output to a while loop
candump ${CAN_INTERFACE} | while read -r line; do
    if [ -n "$line" ]; then
        # If a line is read, update the timestamp
        LAST_DATA_TIMESTAMP=$(date +%s)
        # Provide continuous feedback that things are working
        echo -ne "OK: CAN data is flowing. Last message received at $(date +'%H:%M:%S'). ROS and camera are running...
"
    fi
done &
CANDUMP_PID=$!

# This loop checks if the time since the last message exceeds the timeout
{
while true; do
    CURRENT_TIME=$(date +%s)
    TIME_DIFF=$((CURRENT_TIME - LAST_DATA_TIMESTAMP))
    
    if [ "$TIME_DIFF" -gt "$MONITOR_TIMEOUT" ]; then
        # `echo -e` handles the newline properly before the error message
        echo -e "
[$(date +'%Y-%m-%d %H:%M:%S')] ERROR: No CAN data received for $TIME_DIFF seconds!"
        # Reset timestamp to avoid flooding with error messages
        LAST_DATA_TIMESTAMP=$(date +%s)
    fi
    sleep 1
done
} &
MONITOR_PID=$!

# Wait for the monitor process to exit. If it's killed by the cleanup function, this wait will return.
wait $MONITOR_PID

    fi
done &
CANDUMP_PID=$!

# This loop checks if the time since the last message exceeds the timeout
while true; do
    CURRENT_TIME=$(date +%s)
    TIME_DIFF=$((CURRENT_TIME - LAST_DATA_TIMESTAMP))
    
    if [ "$TIME_DIFF" -gt "$MONITOR_TIMEOUT" ]; then
        # `echo -e` handles the newline properly before the error message
        echo -e "\n[$(date +'%Y-%m-%d %H:%M:%S')] ERROR: No CAN data received for $TIME_DIFF seconds!"
        # Reset timestamp to avoid flooding with error messages
        LAST_DATA_TIMESTAMP=$(date +%s)
    fi
    sleep 1
done
