import rclpy
from rclpy.node import Node
from piper_msgs.msg import PosCmd, PiperStatusMsg
from sensor_msgs.msg import JointState
import time
import inputs
import threading
import math

class XboxToPosNode(Node):
    def __init__(self):
        super().__init__('xbox_to_pos_node')

        self.error_state = False

        # piper_ros용 퍼블리셔 생성
        self.pos_cmd_publisher = self.create_publisher(PosCmd, '/pos_cmd', 10)
        self.joint_ctrl_publisher = self.create_publisher(JointState, 'joint_ctrl_single', 10)

        # arm_status 구독자 생성
        self.arm_status_subscription = self.create_subscription(
            PiperStatusMsg,
            '/arm_status',
            self.arm_status_callback,
            10)

        # 중심점 정의 (로봇의 기본 자세)
        self.center_pose = PosCmd()
        self.center_pose.x = 0.0
        self.center_pose.y = 0.0
        self.center_pose.z = 0.0
        self.center_pose.roll = 0.0
        self.center_pose.pitch = 1.57  # 90도 (라디안)
        self.center_pose.yaw = 0.0
        self.center_pose.gripper = 0.0

        # 초기 상태 (로봇의 시작 위치)
        self.current_pose = PosCmd()
        self.current_pose.x = 0.3
        self.current_pose.y = 0.0
        self.current_pose.z = 0.3
        self.current_pose.roll = 0.0
        self.current_pose.pitch = 1.57  # 90도 (라디안)
        self.current_pose.yaw = 0.0
        self.current_pose.gripper = 0.0

        # 컨트롤러 축과 버튼 상태를 저장하는 딕셔너리
        self.axes = {'ABS_X': 0, 'ABS_Y': 0, 'ABS_RX': 0, 'ABS_RY': 0, 'ABS_HAT0X': 0, 'ABS_Z': 0, 'ABS_RZ': 0}
        self.buttons = {'BTN_TR': 0, 'BTN_TL': 0, 'BTN_SOUTH': 0, 'BTN_EAST': 0, 'BTN_NORTH': 0, 'BTN_WEST': 0}
        
        # 트리거 버튼 눌림 상태
        self.lt_pressed = False
        self.rt_pressed = False

        # 왼쪽 범퍼 토글 상태
        self.lb_toggle_state = False

        self.get_logger().info('Xbox to Piper PosCmd node started.')
        self.print_controls()

        # 컨트롤러 이벤트를 처리하는 별도 스레드 시작
        self.controller_thread = threading.Thread(target=self.process_controller_events)
        self.controller_thread.daemon = True
        self.controller_thread.start()

        # 0.1초마다 현재 자세를 발행하는 타이머 생성
        self.publisher_timer = self.create_timer(0.1, self.publish_pose)

    def arm_status_callback(self, msg):
        """arm_status 토픽 콜백 함수"""
        if msg.arm_status != 0:
            if not self.error_state:
                self.get_logger().warn(f'Arm status is not normal (status: {msg.arm_status}), entering error state. Returning to center.')
                self.error_state = True
            self.reset_joints_to_center()
        else:
            if self.error_state:
                self.get_logger().info('Arm status is normal. Exiting error state.')
                self.error_state = False

    def reset_joints_to_center(self):
        """모든 조인트를 0으로 리셋하는 JointState 메시지를 발행"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        joint_state_msg.position = [0.0] * 7
        joint_state_msg.velocity = [0.0] * 7
        joint_state_msg.effort = [0.0] * 7
        self.joint_ctrl_publisher.publish(joint_state_msg)
        self.get_logger().info('Published JointState to reset arm to center on topic joint_ctrl_single.')

    def print_controls(self):
        """컨트롤러 조작법을 로그로 출력"""
        self.get_logger().info("--- Controls ---")
        self.get_logger().info("Right Stick: X/Y Plane Control")
        self.get_logger().info("Left Stick: Roll/Pitch Control")
        self.get_logger().info("D-Pad Up/Down: Yaw Control")
        self.get_logger().info("Right Bumper: Reset to Center Point")
        self.get_logger().info("Left Bumper: Toggle X offset (+0.3)")
        self.get_logger().info("Left/Right Triggers: Rotate Yaw by 45 degrees")
        self.get_logger().info("A: Z Down, Y: Z Up")
        self.get_logger().info("X: Gripper Close, B: Gripper Open")
        self.get_logger().info("----------------")

    def process_controller_events(self):
        """컨트롤러 이벤트를 지속적으로 처리하는 메서드"""
        try:
            gamepad = inputs.devices.gamepads[0]  # 첫 번째 게임패드 연결
        except IndexError:
            self.get_logger().error("No gamepad found.")
            return

        while rclpy.ok():
            try:
                events = gamepad.read()  # 게임패드 이벤트 읽기
            except EOFError:
                self.get_logger().error("Gamepad disconnected.")
                break
            for event in events:
                if event.ev_type == 'Key':  # 버튼 이벤트
                    self.handle_button_press(event)
                elif event.ev_type == 'Absolute':  # 스틱/축 이벤트
                    self.handle_stick_move(event)

    def handle_button_press(self, event):
        """버튼 입력 처리"""
        if event.code in self.buttons:
            self.buttons[event.code] = event.state

        if event.code == 'BTN_TR': # Right Bumper (RB) - 중심점으로 리셋
            if event.state == 1: # 버튼이 눌렸을 때
                self.reset_to_center()
        
        if event.code == 'BTN_TL': # Left Bumper (LB) - X 오프셋 토글
            if event.state == 1: # 버튼이 눌렸을 때
                if not self.lb_toggle_state:
                    self.current_pose.x += 0.3
                    self.lb_toggle_state = True
                    self.get_logger().info("X offset applied.")
                else:
                    self.current_pose.x -= 0.3
                    self.lb_toggle_state = False
                    self.get_logger().info("X offset removed.")

    def reset_to_center(self):
        """현재 자세를 중심점으로 리셋"""
        self.current_pose.x = self.center_pose.x
        self.current_pose.y = self.center_pose.y
        self.current_pose.z = self.center_pose.z
        self.current_pose.roll = self.center_pose.roll
        self.current_pose.pitch = self.center_pose.pitch
        self.current_pose.yaw = self.center_pose.yaw
        self.current_pose.gripper = self.center_pose.gripper
        self.axes = {'ABS_X': 0, 'ABS_Y': 0, 'ABS_RX': 0, 'ABS_RY': 0, 'ABS_HAT0X': 0, 'ABS_Z': 0, 'ABS_RZ': 0}
        self.buttons = {'BTN_TR': 0, 'BTN_TL': 0, 'BTN_SOUTH': 0, 'BTN_EAST': 0, 'BTN_NORTH': 0, 'BTN_WEST': 0}
        self.get_logger().info("Pose reset to center point.")


    def handle_stick_move(self, event):
        """스틱과 축 입력 처리"""
        trigger_threshold = 30 # 아날로그 트리거 임계값

        if event.code == 'ABS_RZ': # Right Trigger
            is_pressed = event.state > trigger_threshold
            if is_pressed and not self.rt_pressed:
                self.current_pose.yaw -= math.pi / 4 # 우측으로 45도 회전
            self.rt_pressed = is_pressed
            return
        
        if event.code == 'ABS_Z': # Left Trigger
            is_pressed = event.state > trigger_threshold
            if is_pressed and not self.lt_pressed:
                self.current_pose.yaw += math.pi / 4 # 좌측으로 45도 회전
            self.lt_pressed = is_pressed
            return

        if event.code == 'ABS_HAT0X':  # D-pad는 이산값 (-1, 0, 1)
            self.axes[event.code] = event.state
        elif event.code in self.axes:
            # 축 값을 -32768~32767에서 -1.0~1.0으로 정규화
            value = event.state / 32768.0
            self.axes[event.code] = value

    def publish_pose(self):
        """현재 자세를 계산하고 발행하는 메서드"""
        if self.error_state:
            return

        sensitivity = 0.01        # 위치 이동 민감도
        rot_sensitivity = 0.02    # 회전 민감도
        gripper_sensitivity = 0.005  # 그리퍼 민감도

        # 오른쪽 스틱으로 X/Y 평면 제어
        right_stick_lr = self.axes.get('ABS_RX', 0.0)  # 좌우
        right_stick_ud = self.axes.get('ABS_RY', 0.0)  # 상하
        self.current_pose.x -= right_stick_ud * sensitivity
        self.current_pose.y -= right_stick_lr * sensitivity
        
        # A와 Y 버튼으로 Z축 제어
        if self.buttons.get('BTN_SOUTH', 0) == 1: # A 버튼 - Z축 아래로
            self.current_pose.z -= sensitivity
        if self.buttons.get('BTN_WEST', 0) == 1: # Y 버튼 - Z축 위로
            self.current_pose.z += sensitivity

        # X와 B 버튼으로 그리퍼 제어
        if self.buttons.get('BTN_NORTH', 0) == 1: # X 버튼 - 그리퍼 닫기
            self.current_pose.gripper -= gripper_sensitivity
        if self.buttons.get('BTN_EAST', 0) == 1: # B 버튼 - 그리퍼 열기
            self.current_pose.gripper += gripper_sensitivity
        
        # 그리퍼 값을 0.0~0.1 범위로 제한
        self.current_pose.gripper = max(0.0, min(0.1, self.current_pose.gripper))

        # 왼쪽 스틱으로 Roll/Pitch 제어
        left_stick_lr = self.axes.get('ABS_X', 0.0)   # 좌우
        left_stick_ud = self.axes.get('ABS_Y', 0.0)   # 상하
        self.current_pose.roll += left_stick_lr * rot_sensitivity
        self.current_pose.pitch -= left_stick_ud * rot_sensitivity

        # D-pad로 Yaw 제어
        dpad_y = self.axes.get('ABS_HAT0X', 0)
        if dpad_y == -1: # D-pad Up - Yaw 증가
            self.current_pose.yaw += rot_sensitivity
        elif dpad_y == 1: # D-pad Down - Yaw 감소
            self.current_pose.yaw -= rot_sensitivity

        # 계산된 자세를 ROS 토픽으로 발행
        self.pos_cmd_publisher.publish(self.current_pose)


def main(args=None):
    """메인 함수 - 노드 초기화 및 실행"""
    rclpy.init(args=args)
    node = XboxToPosNode()
    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
