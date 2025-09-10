import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

class OperatorSwitchNode(Node):
    def __init__(self):
        super().__init__('operator_switch_node')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'operator_switch',
            self.listener_callback,
            10)
        self.get_logger().info('Operator Switch Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {list(msg.data)}')
        
        if len(msg.data) < 3:
            self.get_logger().warn('Received data has less than 3 elements, ignoring.')
            return

        data0 = msg.data[0]
        data1 = msg.data[1]
        data2 = msg.data[2]

        self.get_logger().info(f'Data[0]: {data0}, Data[1]: {data1}, Data[2]: {data2}')

        # --- 여기에 data 값에 따른 동작을 추가하세요 ---
        if data0 == 1:
            self.get_logger().info('Action for data[0] == 1 triggered.')
            # 예: 로봇 정지 명령
        
        if data1 > 100:
            self.get_logger().info('Action for data[1] > 100 triggered.')
            # 예: 특정 속도로 이동

        if data2 == 255:
            self.get_logger().info('Action for data[2] == 255 triggered.')
            # 예: 비상 정지
        # ----------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    operator_switch_node = OperatorSwitchNode()
    rclpy.spin(operator_switch_node)
    operator_switch_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
