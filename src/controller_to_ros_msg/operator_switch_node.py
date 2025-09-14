import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from piper_msgs.msg import PiperStatusMsg
from sensor_msgs.msg import JointState
import copy
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

        self.center_joint_state_msg = JointState()
        self.center_joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.center_joint_state_msg.position = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        self.center_joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0]
        self.center_joint_state_msg.effort = [0.0] * 6

        self.current_joint_state_msg = JointState()
        self.current_joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.current_joint_state_msg.position = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        self.current_joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0]
        self.current_joint_state_msg.effort = [0.0] * 6

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
            self.current_joint_state_msg.velocity[6] = 30.0
            self.joint_ctrl_publisher.publish(self.center_joint_state_msg)
                        # 깊은 복사로 값만 복사 (참조가 아닌)
            self.current_joint_state_msg.position = copy.deepcopy(self.center_joint_state_msg.position)
            self.current_joint_state_msg.velocity = copy.deepcopy(self.center_joint_state_msg.velocity)
            self.current_joint_state_msg.effort = copy.deepcopy(self.center_joint_state_msg.effort)
        elif pose_name == 'gripper_open':
            self.current_joint_state_msg.position[6] = 0.5
            self.center_joint_state_msg.position[6] = 0.5
            self.current_joint_state_msg.velocity[6] = 15.0
            self.joint_ctrl_publisher.publish(self.current_joint_state_msg)
        elif pose_name == 'gripper_close':
            self.current_joint_state_msg.position[6] = 0.0
            self.center_joint_state_msg.position[6] = 0.0
            self.current_joint_state_msg.velocity[6] = 15.0
            self.joint_ctrl_publisher.publish(self.current_joint_state_msg)
        elif pose_name == 'behind':
            self.current_joint_state_msg.position[0] = -2.5758159280000004
            self.current_joint_state_msg.position[1] = 0.528518312
            self.current_joint_state_msg.position[2] = -0.38394244000000005
            self.current_joint_state_msg.position[3] = 1.465435552
            self.current_joint_state_msg.position[4] = -0.652649816
            self.current_joint_state_msg.position[5] = 0.148151892
            self.current_joint_state_msg.velocity[6] = 30.0
            self.joint_ctrl_publisher.publish(self.current_joint_state_msg)
        elif pose_name == 'front':         
            self.current_joint_state_msg.position[0] = 0.019118624000000004
            self.current_joint_state_msg.position[1] = 1.9516696080000002
            self.current_joint_state_msg.position[2] = -2.165341164
            self.current_joint_state_msg.position[3] = -0.059588704
            self.current_joint_state_msg.position[4] = 0.20589153200000002
            self.current_joint_state_msg.position[5] = 1.641637396
            self.current_joint_state_msg.velocity[6] = 20.0
            self.joint_ctrl_publisher.publish(self.current_joint_state_msg)
            # self.current_joint_state_msg.position[0] = 0.017670771999999998
            # self.current_joint_state_msg.position[1] = 1.9704044640000002
            # self.current_joint_state_msg.position[2] = -1.360754108
            # self.current_joint_state_msg.position[3] = -0.025555460000000002
            # self.current_joint_state_msg.position[4] = -0.575721776
            # self.current_joint_state_msg.position[5] = 1.64113152
            # self.current_joint_state_msg.position[0] = -0.03148642
            # self.current_joint_state_msg.position[1] = 2.075818556
            # self.current_joint_state_msg.position[2] = -1.3262673200000001
            # self.current_joint_state_msg.position[3] = 0.156839004
            # self.current_joint_state_msg.position[4] = -0.31172428
            # self.current_joint_state_msg.position[5] = 1.529908576
            # self.current_joint_state_msg.velocity[6] = 20.0
            # self.joint_ctrl_publisher.publish(self.current_joint_state_msg)

        self.get_logger().info(f'Published JointState to set {pose_name} on topic joint_ctrl_single.')


def main(args=None):
    rclpy.init(args=args)
    operator_switch_node = OperatorSwitchNode()
    rclpy.spin(operator_switch_node)
    operator_switch_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
