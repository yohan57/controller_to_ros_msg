#!/usr/bin/env python3
# -*-coding:utf8-*- 

import time
import math

# piper_sdk가 설치되어 있어야 합니다.
# pip install piper_sdk
try:
    from piper_sdk import C_PiperInterface
except ImportError:
    print("Error: piper_sdk is not installed. Please install it using 'pip install piper_sdk'")
    exit(1)

# --- 설정 ---
CAN_PORT = 'can0'
# 라디안을 SDK가 사용하는 정수 값으로 변환하기 위한 상수
RAD_TO_SDK = 180.0 / math.pi * 1000.0

# operator_switch_node.py에서 가져온 사전 정의된 포즈 (라디안 단위)
# [joint1, joint2, joint3, joint4, joint5, joint6]
PRESET_POSES = {
    "center": [0.0, 0.0, 0.0, 1.57, 0.0, 0.0],
    "behind": [-2.5758, 0.5285, -0.3839, 1.4654, -0.6526, 0.1481],
    "front": [-0.0314, 2.0758, -1.3262, 0.1568, -0.3117, 1.5299],
}

class NonRosPiperControl:
    def __init__(self, can_port=CAN_PORT):
        """PiperInterface를 초기화하고 로봇 팔을 활성화합니다."""
        print(f"Initializing Piper on CAN port: {can_port}")
        self.piper = C_PiperInterface(can_name=can_port)
        if not self.piper.ConnectPort():
            print(f"Error: Failed to connect to CAN port '{can_port}'. Exiting.")
            exit(1)
        print("Successfully connected to CAN port.")

        print("Enabling arm...")
        self.piper.EnableArm(7) # 모든 조인트 활성화
        time.sleep(1)  # 활성화를 위한 잠시 대기

        # 활성화 상태 확인 (첫 번째 모터 기준)
        if not self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status:
            print("Warning: Failed to confirm arm enable status. Please check connection and power.")
        else:
            print("Arm enabled successfully.")
        
        # 현재 조인트 상태를 'center' 포즈로 초기화
        self.current_joint_positions = list(PRESET_POSES["center"])
        self.current_gripper_position = 0.0

    def set_pose(self, pose_name):
        """사전 정의된 포즈로 로봇 팔을 이동시킵니다."""
        if pose_name not in PRESET_POSES:
            print(f"Error: Pose '{pose_name}' not defined.")
            return

        print(f"Setting pose to: {pose_name}")
        self.current_joint_positions = list(PRESET_POSES[pose_name])
        self._send_joint_command()

    def set_gripper(self, state):
        """그리퍼를 열거나 닫습니다."""
        if state == 'open':
            print("Setting gripper to OPEN")
            self.current_gripper_position = 0.5  # operator_switch_node.py의 열림 값
        elif state == 'close':
            print("Setting gripper to CLOSE")
            self.current_gripper_position = 0.0
        else:
            print(f"Unknown gripper state: {state}")
            return
        self._send_gripper_command()

    def _send_joint_command(self):
        """현재 조인트 위치를 SDK 형식으로 변환하여 전송합니다."""
        sdk_positions = [int(p * RAD_TO_SDK) for p in self.current_joint_positions]
        print(f"Sending JointCtrl command: {sdk_positions}")
        self.piper.JointCtrl(*sdk_positions)

    def _send_gripper_command(self):
        """현재 그리퍼 위치를 SDK 형식으로 변환하여 전송합니다."""
        # 위치 값은 0~80000 사이, 속도는 1000, 모드는 위치 제어(0x01)
        gripper_pos_sdk = int(self.current_gripper_position * 80000) 
        print(f"Sending GripperCtrl command: {gripper_pos_sdk}")
        self.piper.GripperCtrl(abs(gripper_pos_sdk), 1000, 0x01, 0)

    def shutdown(self):
        """로봇 팔을 비활성화하고 연결을 해제합니다."""
        print("Disabling arm and shutting down.")
        self.piper.DisableArm(7)
        # self.piper.DisconnectPort() # DisconnectPort가 SDK에 없을 수 있음

def main():
    """메인 실행 루프"""
    controller = NonRosPiperControl()
    last_inputs = (None, None, None)

    try:
        while True:
            print("\n-----------------------------------------------------")
            print("Enter operator switch values (0 or 1). Press Ctrl+C to exit.")
            try:
                in0_str = input("Enter value for switch 0 (gripper: 1=open, 0=close): ")
                in1_str = input("Enter value for switch 1 (move to front): ")
                in2_str = input("Enter value for switch 2 (move to behind): ")

                data0 = int(in0_str)
                data1 = int(in1_str)
                data2 = int(in2_str)
                current_inputs = (data0, data1, data2)

            except ValueError:
                print("\nError: Invalid input. Please enter only 0 or 1.")
                continue

            # 입력 값이 변경되었을 때만 명령 실행
            if current_inputs == last_inputs:
                print("No change in inputs, skipping command.")
                continue
            
            # 1. 그리퍼 제어 (data0)
            if data0 != last_inputs[0]:
                if data0 == 1:
                    controller.set_gripper('open')
                else:
                    controller.set_gripper('close')
            
            # 잠시 대기 후 포즈 변경
            time.sleep(0.2)

            # 2. 팔 포즈 제어 (data1, data2)
            if (data1, data2) != (last_inputs[1], last_inputs[2]):
                if data1 == 1 and data2 == 0:
                    controller.set_pose('front')
                elif data2 == 1 and data1 == 0:
                    controller.set_pose('behind')
                elif data2 == 0 and data1 == 0:
                    controller.set_pose('center')
            
            last_inputs = current_inputs

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        controller.shutdown()

if __name__ == '__main__':
    main()
