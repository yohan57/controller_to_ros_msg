import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from piper_msgs.msg import PiperStatusMsg
from sensor_msgs.msg import JointState

PRESET_POSES = {
    "center": [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0],
    "behind": [-2.5758, 0.5285, -0.3839, 1.4654, -0.6526, 0.1481, 0.0],
    "front": [-0.0314, 2.0758, -1.3262, 0.1568, -0.3117, 1.5299, 0.0],
}

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
        # self.pos_cmd_publisher = self.create_publisher(PosCmd, '/pos_cmd', 10)
        self.joint_ctrl_publisher = self.create_publisher(JointState, 'joint_ctrl_single', 10)
        # self.joint_ctrl_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.joint_state_ctrl_msg = JointState()
        self.joint_state_ctrl_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_state_ctrl_msg.position = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        self.joint_state_ctrl_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0]
        self.joint_state_ctrl_msg.effort = [0.0] * 6

        # 이전 데이터 값 초기화
        self.last_data0 = None
        self.last_data1 = None
        self.last_data2 = None

        self.error_state = False

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

        if data1 == 1 and data2 == 0:
            self.get_logger().info('Action for data[1] == 1 triggered.')
            self.set_preset_pose('front')
        if data2 == 1 and data1 == 0:
            self.get_logger().info('Action for data[2] == 1 triggered.')
            self.set_preset_pose('behind')
        if data2 == 0 and data1 == 0:
            self.get_logger().info('Action for data[2] == 0 triggered.')
            self.set_preset_pose('center')

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
        self.joint_ctrl_publisher.publish(self.center_joint_state_msg)
        self.get_logger().info('Published JointState to reset arm to center on topic joint_ctrl_single.')
        
    def set_preset_pose(self, pose_name):
        """사전 정의된 위치로 이동하는 PosCmd 메시지를 발행"""
        if pose_name == 'center':
            self.joint_state_ctrl_msg.position = PRESET_POSES["center"]
            self.joint_state_ctrl_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0]
            self.joint_state_ctrl_msg.effort = [0.0] * 6
            self.joint_ctrl_publisher.publish(self.joint_state_ctrl_msg)
        elif pose_name == 'gripper_open':
            self.joint_state_ctrl_msg.position[6] = 0.5
            self.joint_state_ctrl_msg.velocity[6] = 15.0
            self.joint_ctrl_publisher.publish(self.joint_state_ctrl_msg)
        elif pose_name == 'gripper_close':
            self.joint_state_ctrl_msg.position[6] = 0.0
            self.joint_state_ctrl_msg.velocity[6] = 15.0
            self.joint_ctrl_publisher.publish(self.joint_state_ctrl_msg)
        elif pose_name == 'behind':
            self.joint_state_ctrl_msg.position = PRESET_POSES["behind"]
            self.joint_state_ctrl_msg.velocity[6] = 30.0
            self.joint_ctrl_publisher.publish(self.joint_state_ctrl_msg)
        elif pose_name == 'front':         
            self.joint_state_ctrl_msg.position = PRESET_POSES["front"]
            self.joint_state_ctrl_msg.velocity[6] = 20.0
            self.joint_ctrl_publisher.publish(self.joint_state_ctrl_msg)

        self.get_logger().info(f'Published JointState to set {pose_name} on topic joint_ctrl_single.')


def main(args=None):
    rclpy.init(args=args)
    operator_switch_node = OperatorSwitchNode()
    rclpy.spin(operator_switch_node)
    operator_switch_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
