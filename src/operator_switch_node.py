import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from piper_msgs.msg import PosCmd, PiperStatusMsg
from sensor_msgs.msg import JointState

class OperatorSwitchNode(Node):
    def __init__(self):
        super().__init__('operator_switch_node')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'operator_switch',
            self.listener_callback,
            10)
        self.get_logger().info('Operator Switch Node has been started.')
        self.arm_status_subscription = self.create_subscription(
            PiperStatusMsg,
            '/arm_status',
            self.arm_status_callback,
            10)

        # piper_ros용 퍼블리셔 생성
        self.pos_cmd_publisher = self.create_publisher(PosCmd, '/pos_cmd', 10)
        self.joint_ctrl_publisher = self.create_publisher(JointState, 'joint_ctrl_single', 10)

        # 초기 상태 (로봇의 시작 위치)
        self.current_pose = PosCmd()
        self.current_pose.x = 0.055
        self.current_pose.y = 0.0
        self.current_pose.z = 0.21
        self.current_pose.roll = 0.0
        self.current_pose.pitch = 1.57  # 90도 (라디안)
        self.current_pose.yaw = 0.0
        self.current_pose.gripper = 0.0

        # 이전 데이터 값 초기화
        self.last_data0 = None
        self.last_data1 = None
        self.last_data2 = None

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {list(msg.data)}')
        
        if len(msg.data) < 3:
            self.get_logger().warn('Received data has less than 3 elements, ignoring.')
            return

        data0 = msg.data[0]
        data1 = msg.data[1]
        data2 = msg.data[2]

        self.get_logger().info(f'Data[0]: {data0}, Data[1]: {data1}, Data[2]: {data2}')

        # 값이 기존 값에서 변할 때만 동작하도록 수정
        if data0 != self.last_data0:
            self.last_data0 = data0
            if data0 == 1:
                self.get_logger().info('Action for data[0] == 1 triggered.')
                # 0 -> gripper open , 1 -> gripper close
                self.set_preset_pose('gripper_open')
            else:
                self.get_logger().info('Action for data[0] == 0 triggered.')
                self.set_preset_pose('gripper_close')

        if data1 == 1:
            self.set_preset_pose('behind')

        if data2 == 1:
            self.set_preset_pose('front')

    def arm_status_callback(self, msg):
        """arm_status 토픽 콜백 함수"""
        if msg.arm_status != 0:
            if not self.error_state:
                self.get_logger().warn(f'Arm status is not normal (status: {msg.arm_status}), entering error state. Returning to center.')
                self.error_state = True
            self.reset_joints_to_center()
            self.reset_to_center()
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
        joint_state_msg.velocity = []
        joint_state_msg.effort = []
        self.joint_ctrl_publisher.publish(joint_state_msg)
        self.get_logger().info('Published JointState to reset arm to center on topic joint_ctrl_single.')
        
    def reset_to_center(self):
        """초기 위치로 이동하는 PosCmd 메시지를 발행"""
        self.current_pose.x = 0.055
        self.current_pose.y = 0.0
        self.current_pose.z = 0.21
        self.current_pose.roll = 0.0
        self.current_pose.pitch = 1.57
        self.current_pose.yaw = 0.0
        self.current_pose.gripper = 0.0
        self.pos_cmd_publisher.publish(self.current_pose)
        self.get_logger().info('Published PosCmd to reset arm to center on topic pos_cmd.')


    def set_preset_pose(self, pose_name):
        """사전 정의된 위치로 이동하는 PosCmd 메시지를 발행"""
        if pose_name == 'center':
            self.current_pose.x = 0.055
            self.current_pose.y = 0.0
            self.current_pose.z = 0.21
            self.current_pose.roll = 0.0
            self.current_pose.pitch = 1.57
            self.current_pose.yaw = 0.0
            self.current_pose.gripper = 0.0
        elif pose_name == 'gripper_open':
            self.current_pose.gripper = 1.0
        elif pose_name == 'gripper_close':
            self.current_pose.gripper = 0.0
        elif pose_name == 'behind':
            self.current_pose.x = 0.055
            self.current_pose.y = 0.0
            self.current_pose.z = 0.21
            self.current_pose.roll = 0.0
            self.current_pose.pitch = 1.57
            self.current_pose.yaw = 0.0
            self.current_pose.gripper = 0.0
        elif pose_name == 'front':         
            self.current_pose.x = 0.055
            self.current_pose.y = 0.0
            self.current_pose.z = 0.21
            self.current_pose.roll = 0.0
            self.current_pose.pitch = 1.57
            self.current_pose.yaw = 0.0
            self.current_pose.gripper = 0.0
        self.pos_cmd_publisher.publish(self.current_pose)
        self.get_logger().info(f'Published PosCmd to set {pose_name} on topic pos_cmd.')


def main(args=None):
    rclpy.init(args=args)
    operator_switch_node = OperatorSwitchNode()
    rclpy.spin(operator_switch_node)
    operator_switch_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
