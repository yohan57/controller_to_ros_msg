import rclpy
from rclpy.node import Node
from piper_msgs.msg import PosCmd
from sensor_msgs.msg import JointState

def get_float_input(prompt):
    """Safely get a float input from the user."""
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Invalid input. Please enter a number.")

class ManualPiperPublisher(Node):
    def __init__(self):
        super().__init__('manual_piper_publisher')
        self.pos_cmd_publisher = self.create_publisher(PosCmd, '/pos_cmd', 10)
        self.joint_ctrl_publisher = self.create_publisher(JointState, 'joint_ctrl_single', 10)
        
        self.last_message = None
        self.last_message_type = None  # 'pose' or 'joint'

        self.get_logger().info('Manual Piper Publisher node started.')

    def run(self):
        """Main loop to get user input and publish messages."""
        while rclpy.ok():
            self.print_instructions()
            command = input("Enter command: ").lower().strip()

            if command == 'quit':
                break
            elif command == 'resend':
                self.handle_resend()
            elif command == 'pose':
                self.handle_pose()
            elif command == 'joint':
                self.handle_joint()
            else:
                self.get_logger().warn(f"Unknown command: {command}")

    def print_instructions(self):
        print("\n-----------------------------------------------------")
        print("Enter a command:")
        print("  'pose'   - to send a new Pose (PosCmd) command")
        print("  'joint'  - to send a new Joint (JointState) command")
        print("  'resend' - to send the last command again")
        print("  'quit'   - to exit the program")
        print("-----------------------------------------------------")

    def handle_resend(self):
        if self.last_message is None:
            self.get_logger().warn("No previous command to resend.")
            return

        if self.last_message_type == 'pose':
            self.pos_cmd_publisher.publish(self.last_message)
            self.get_logger().info("Resending last PosCmd.")
        elif self.last_message_type == 'joint':
            # Update timestamp for JointState before resending
            self.last_message.header.stamp = self.get_clock().now().to_msg()
            self.joint_ctrl_publisher.publish(self.last_message)
            self.get_logger().info("Resending last JointState command.")
        
        self.get_logger().info(f"Published: {self.last_message}")

    def handle_pose(self):
        msg = PosCmd()
        print("--- Enter Pose (PosCmd) Values ---")
        msg.x = get_float_input("Enter X: ")
        msg.y = get_float_input("Enter Y: ")
        msg.z = get_float_input("Enter Z: ")
        msg.roll = get_float_input("Enter Roll: ")
        msg.pitch = get_float_input("Enter Pitch: ")
        msg.yaw = get_float_input("Enter Yaw: ")
        msg.gripper = get_float_input("Enter Gripper (0.0 open to 1.0 closed): ")

        self.pos_cmd_publisher.publish(msg)
        self.get_logger().info(f"Publishing PosCmd: {msg}")
        self.last_message = msg
        self.last_message_type = 'pose'

    def handle_joint(self):
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        
        print("--- Enter Joint (JointState) Positions ---")
        positions = []
        for joint_name in msg.name:
            pos = get_float_input(f"Enter position for {joint_name}: ")
            positions.append(pos)
        
        msg.position = [float(p) for p in positions]
        msg.header.stamp = self.get_clock().now().to_msg()

        self.joint_ctrl_publisher.publish(msg)
        self.get_logger().info(f"Publishing JointState: {msg}")
        self.last_message = msg
        self.last_message_type = 'joint'

def main(args=None):
    rclpy.init(args=args)
    node = ManualPiperPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
