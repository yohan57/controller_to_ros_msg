#!/usr/bin/env python3
# -*-coding:utf8-*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import time
import math

try:
    from piper_sdk import C_PiperInterface
except ImportError:
    print("Error: piper_sdk is not installed. Please install it using 'pip install piper_sdk'")
    exit(1)

# --- 상수 정의 ---
CAN_PORT = 'can0'
RAD_TO_SDK = 180.0 / math.pi * 1000.0

# 사전 정의된 포즈 (라디안)
PRESET_POSES = {
    "center": [0.0, 0.0, 0.0, 1.57, 0.0, 0.0],
    "behind": [-2.5758, 0.5285, -0.3839, 1.4654, -0.6526, 0.1481],
    "front": [-0.0314, 2.0758, -1.3262, 0.1568, -0.3117, 1.5299],
}

class RosToPiperDirectNode(Node):
    """ROS 토픽을 구독하고 Piper SDK를 통해 직접 로봇을 제어하는 노드"""
    def __init__(self):
        super().__init__('ros_to_piper_direct_node')
        
        # ROS Subscriber 설정
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'operator_switch',
            self.listener_callback,
            10)
        
        # Piper SDK 초기화
        self.get_logger().info(f"Initializing Piper on CAN port: {CAN_PORT}")
        self.piper = C_PiperInterface(can_name=CAN_PORT)
        if not self.piper.ConnectPort():
            self.get_logger().error(f"Failed to connect to CAN port '{CAN_PORT}'. Exiting.")
            rclpy.shutdown()
            exit(1)
        self.get_logger().info("Successfully connected to CAN port.")

        self.get_logger().info("Enabling arm...")
        self.piper.EnableArm(7)
        time.sleep(1)
        
        if not self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status:
            self.get_logger().warn("Failed to confirm arm enable status. Please check connection and power.")
        else:
            self.get_logger().info("Arm enabled successfully.")

        # 상태 변수
        self.current_joint_positions = list(PRESET_POSES["center"])
        self.current_gripper_position = 0.0
        self.last_ros_data = (None, None, None)

        self.get_logger().info('ROS to Piper Direct Bridge node has been started.')

    def listener_callback(self, msg):
        """'operator_switch' 토픽의 콜백 함수"""
        if len(msg.data) < 3:
            self.get_logger().warn('Received data has less than 3 elements, ignoring.')
            return

        data0, data1, data2 = msg.data[0], msg.data[1], msg.data[2]
        current_ros_data = (data0, data1, data2)

        if current_ros_data == self.last_ros_data:
            return
        
        self.get_logger().info(f"New command received: {current_ros_data}")

        # 1. 그리퍼 제어 (data0)
        if self.last_ros_data[0] is None or data0 != self.last_ros_data[0]:
            if data0 == 1:
                self.set_gripper('open')
            else:
                self.set_gripper('close')
        
        # 2. 팔 포즈 제어 (data1, data2)
        if self.last_ros_data[1] is None or (data1, data2) != (self.last_ros_data[1], self.last_ros_data[2]):
            if data1 == 1 and data2 == 0:
                self.set_pose('front')
            elif data2 == 1 and data1 == 0:
                self.set_pose('behind')
            elif data2 == 0 and data1 == 0:
                self.set_pose('center')
        
        self.last_ros_data = current_ros_data

    def set_pose(self, pose_name):
        if pose_name not in PRESET_POSES:
            self.get_logger().error(f"Pose '{pose_name}' not defined.")
            return

        self.get_logger().info(f"Setting pose to: {pose_name}")
        self.current_joint_positions = list(PRESET_POSES[pose_name])
        self._send_joint_command()

    def set_gripper(self, state):
        if state == 'open':
            self.get_logger().info("Setting gripper to OPEN")
            self.current_gripper_position = 0.5
        elif state == 'close':
            self.get_logger().info("Setting gripper to CLOSE")
            self.current_gripper_position = 0.0
        else:
            self.get_logger().warn(f"Unknown gripper state: {state}")
            return
        self._send_gripper_command()

    def _send_joint_command(self):
        sdk_positions = [int(p * RAD_TO_SDK) for p in self.current_joint_positions]
        self.get_logger().info(f"Sending JointCtrl command: {sdk_positions}")
        self.piper.JointCtrl(*sdk_positions)

    def _send_gripper_command(self):
        # 위치 값은 0~80000 사이, 속도는 1000, 모드는 위치 제어(0x01)
        gripper_pos_sdk = int(self.current_gripper_position * 80000)
        self.get_logger().info(f"Sending GripperCtrl command: {gripper_pos_sdk}")
        self.piper.GripperCtrl(abs(gripper_pos_sdk), 1000, 0x01, 0)

    def shutdown_piper(self):
        self.get_logger().info("Disabling arm.")
        self.piper.DisableArm(7)

def main(args=None):
    rclpy.init(args=args)
    node = RosToPiperDirectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.shutdown_piper()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
