#!/usr/bin/env python3
# -*-coding:utf8-*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from autonomy_ros2_message.msg import DetectionBoundingBoxArray
from sensor_msgs.msg import JointState
import time
import math
import threading
try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Error: piper_sdk is not installed. Please install it using 'pip install piper_sdk'")
    exit(1)

# --- Constants ---
CAN_PORT = 'can0'
# Conversion factors from ROS units to SDK units
RAD_TO_SDK = 180.0 / math.pi * 1000.0
M_TO_SDK = 1000.0 * 1000.0

class PaiCommanderNode(Node):
    """
    PAI 명령을 수신하여 Piper 로봇 팔을 SDK를 통해 직접 제어하는 노드.
    """
    def __init__(self):
        super().__init__('pai_commander_node')

        # --- Piper SDK Initialization ---
        self.get_logger().info(f"Initializing Piper on CAN port: {CAN_PORT}")
        # judge_flag=False for environments where standard checks might fail
        self.piper = C_PiperInterface_V2(can_name=CAN_PORT, judge_flag=False)
        if not self.piper.get_connect_status():
             self.piper.ConnectPort()

        if not self.piper.get_connect_status():
            self.get_logger().error(f"Failed to connect to CAN port '{CAN_PORT}'. Exiting.")
            rclpy.shutdown()
            exit(1)
        self.get_logger().info("Successfully connected to CAN port.")

        self.get_logger().info("Enabling arm...")
        # Loop until the arm is confirmed to be enabled
        while not all(self.piper.GetArmEnableStatus()):
             self.piper.EnableArm(7)
             time.sleep(0.5)
        self.get_logger().info("Arm enabled successfully.")
        # --- End of SDK Initialization ---

        # ROS Subscribers
        self.create_subscription(
            DetectionBoundingBoxArray,
            'ev_detection',
            self.ev_detection_callback,
            10)
        self.create_subscription(
            String,
            'pai_command',
            self.pai_command_callback,
            10)

        # State variables
        self.latest_detection = None
        self.current_joint_state = JointState()

        # Predefined Joint Poses (in radians)
        self.CENTER_POSE = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        self.TAKE_STUFF_POSE = [-2.5758159280000004, 0.528518312, -0.38394244000000005, 1.465435552, -0.652649816, 0.148151892, 0.0]
        self.GIVE_STUFF_POSE = [0.019118624000000004, 1.9516696080000002, -2.165341164, -0.059588704, 0.20589153200000002, 1.641637396, 0.0]
        self.EV_UP_EASY_POSE = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        self.EV_UP_PRESSED_POSE = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        self.EV_DOWN_EASY_POSE = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        self.EV_DOWN_PRESSED_POSE = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]

        self.get_logger().info('Pai Commander Node with SDK has been started.')

        self.get_joint_states_thread = threading.Thread(target=self.get_joint_states)
        self.get_arm_status_thread = threading.Thread(target=self.get_arm_status)

    def shutdown_hook(self):
        """Gracefully shutdown the piper arm."""
        self.get_logger().info("Shutting down, disabling arm...")
        self.piper.DisableArm(7)
        self.piper.DisconnectPort()

    def ev_detection_callback(self, msg):
        """ev_detection topic callback"""
        self.latest_detection = msg

    def pai_command_callback(self, msg):
        """pai_command topic callback"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        if command == "ev_up":
            self.move_to_ev_object(find_highest=True)
        elif command == "ev_down":
            self.move_to_ev_object(find_highest=False)
        elif command == "ev_up_easy":
            self.set_joint_pose(self.EV_UP_EASY_POSE)
            self.control_gripper(open_gripper=False)
            self.set_joint_pose(self.EV_UP_PRESSED_POSE)
        elif command == "ev_down_easy":
            self.set_joint_pose(self.EV_DOWN_EASY_POSE)
            self.control_gripper(open_gripper=False)
            self.set_joint_pose(self.EV_DOWN_PRESSED_POSE)
        elif command == "gripper_open":
            self.control_gripper(open_gripper=True)
        elif command == "gripper_close":
            self.control_gripper(open_gripper=False)
        elif command == "take_coffee":
            self.set_joint_pose(self.TAKE_STUFF_POSE)
        elif command == "give_coffee":
            self.set_joint_pose(self.GIVE_STUFF_POSE)
        elif command == "center":
            self.set_joint_pose(self.CENTER_POSE)
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def move_to_ev_object(self, find_highest):
        """Moves the arm based on EV detection data using EndPoseCtrl."""
        if self.latest_detection is None or not self.latest_detection.boxes:
            self.get_logger().warn("No EV detection data available.")
            return

        target_box = None
        if find_highest:
            target_box = max(self.latest_detection.boxes, key=lambda box: box.centroid.z)
            self.get_logger().info(f"Moving to highest object at z: {target_box.centroid.z}")
        else:
            target_box = min(self.latest_detection.boxes, key=lambda box: box.centroid.z)
            self.get_logger().info(f"Moving to lowest object at z: {target_box.centroid.z}")

        if target_box:
            # Convert meters to the SDK's expected format (0.001mm) -> multiply by 1e6
            x = int(target_box.centroid.x * M_TO_SDK)
            y = int(target_box.centroid.y * M_TO_SDK)
            z = int(target_box.centroid.z * M_TO_SDK)
            
            # Convert radians to the SDK's expected format (0.001 degrees)
            rx = int(0.0 * RAD_TO_SDK)
            ry = int(1.57 * RAD_TO_SDK)
            rz = int(0.0 * RAD_TO_SDK)

            self.get_logger().info(f"Sending EndPoseCtrl to: x={x}, y={y}, z={z}")
            self.piper.MotionCtrl_2(ctrl_mode=0x01, move_mode=0x00, move_spd_rate_ctrl=50) # MOVE_P
            self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)

    def control_gripper(self, open_gripper):
        """Opens or closes the gripper using GripperCtrl."""
        # 50mm open, 0mm close. SDK unit is 0.001mm.
        gripper_pos_sdk = 50000 if open_gripper else 0
        
        if open_gripper:
            self.get_logger().info("Opening gripper.")
        else:
            self.get_logger().info("Closing gripper.")
            
        # effort=1000, mode=0x01 (enable)
        self.piper.GripperCtrl(gripper_pos_sdk, 1000, 0x01, 0)

    def set_joint_pose(self, pose):
        """Moves the arm to a predefined joint pose using JointCtrl."""
        if len(pose) < 6:
            self.get_logger().error(f"Invalid pose with {len(pose)} elements provided.")
            return

        # Convert joint positions from radians to SDK format
        joint_positions = [int(p * RAD_TO_SDK) for p in pose[:6]]
        
        # Handle gripper separately if specified
        gripper_position = 0
        if len(pose) > 6:
            # Assuming the gripper value is a ratio 0.0 (closed) to 1.0 (fully open at 70mm)
            gripper_position = int(pose[6] * 70000)

        self.get_logger().info(f"Sending JointCtrl for pose: {joint_positions}")
        self.piper.MotionCtrl_2(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=30) # MOVE_J
        self.piper.JointCtrl(*joint_positions)
        
        if len(pose) > 6:
            self.piper.GripperCtrl(gripper_position, 1000, 0x01, 0)

    def get_joint_states(self):
        """Get the current joint states from the Piper robot."""
        while rclpy.ok():
            self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
            self.current_joint_state.position = self.piper.GetJointStates()
            self.current_joint_state.velocity = [self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000,
                                                 self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000,
                                                 self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000,
                                                 self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000,
                                                 self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000,
                                                 self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000]
            self.current_joint_state.effort = [self.piper.GetArmHighSpdInfoMsgs().motor_1.effort/1000,
                                               self.piper.GetArmHighSpdInfoMsgs().motor_2.effort/1000,
                                               self.piper.GetArmHighSpdInfoMsgs().motor_3.effort/1000,
                                               self.piper.GetArmHighSpdInfoMsgs().motor_4.effort/1000,
                                               self.piper.GetArmHighSpdInfoMsgs().motor_5.effort/1000,
                                               self.piper.GetArmHighSpdInfoMsgs().motor_6.effort/1000]
            self.current_joint_state.effort = [self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000]
            time.sleep(0.1)

    def get_arm_status(self):
        """Get the current arm status from the Piper robot."""
        while self.piper.get_connect_status():
            if self.piper.GetArmStatus().arm_status.arm_status != 0x00:
                self.get_logger().error(f"Arm status: {self.piper.GetArmStatus().arm_status.arm_status}")
                self.get_logger().error("Set arm status to center pose")
                self.set_joint_pose(self.CENTER_POSE)
        
    def force_reset_piper(self):
        """Force reset the Piper robot."""
        if self.piper.get_connect_status():
            while not all(self.piper.GetArmEnableStatus()):
                self.piper.EnableArm(7)
                time.sleep(0.5)

            self.piper.MotionCtrl_2(ctrl_mode=0x00, move_mode=0x00, move_spd_rate_ctrl=0)
            self.piper.MotionCtrl_2(ctrl_mode=0x01, move_mode=0x00, move_spd_rate_ctrl=50)


def main(args=None):
    rclpy.init(args=args)
    node = PaiCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()