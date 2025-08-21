import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from piper_msgs.msg import PosCmd
import time

class XboxToPosNode(Node):
    def __init__(self):
        super().__init__('xbox_to_pos_node')

        # Publisher for piper_ros
        self.pos_cmd_publisher = self.create_publisher(PosCmd, '/pos_cmd', 10)

        # Subscriber for Xbox controller
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        # Initial state
        self.current_pose = PosCmd()
        self.current_pose.x = 0.3
        self.current_pose.y = 0.0
        self.current_pose.z = 0.3
        self.current_pose.roll = 0.0
        self.current_pose.pitch = 1.57  # 90 degrees
        self.current_pose.yaw = 0.0
        self.current_pose.gripper = 0.0

        # Mode state for sticks
        self.right_stick_modes = ['xy', 'xz', 'yz']
        self.left_stick_modes = ['roll_pitch', 'pitch_yaw', 'yaw_roll']
        self.right_stick_mode_index = 0
        self.left_stick_mode_index = 0

        # Double-press detection state
        self.rb_last_press_time = 0
        self.lb_last_press_time = 0
        self.double_press_interval = 0.3  # seconds

        self.get_logger().info('Xbox to Piper PosCmd node started.')
        self.print_controls()

    def print_controls(self):
        self.get_logger().info("--- Controls ---")
        self.get_logger().info(f"Right Stick Mode: {self.right_stick_modes[self.right_stick_mode_index].upper()} -> Left/Right: {self.right_stick_modes[self.right_stick_mode_index][0]}, Up/Down: {self.right_stick_modes[self.right_stick_mode_index][1]}")
        self.get_logger().info(f"Left Stick Mode: {self.left_stick_modes[self.left_stick_mode_index].upper()} -> Left/Right: {self.left_stick_modes[self.left_stick_mode_index].split('_')[1]}, Up/Down: {self.left_stick_modes[self.left_stick_mode_index].split('_')[0]}")
        self.get_logger().info("RB (Right Bumper): Change Right Stick Mode")
        self.get_logger().info("LB (Left Bumper): Change Left Stick Mode")
        self.get_logger().info("----------------")

    def joy_callback(self, msg: Joy):
        # --- Button Press Logic for Mode Switching ---
        # Right Bumper (RB) - button 5
        if msg.buttons[5] == 1:
            current_time = time.time()
            if (current_time - self.rb_last_press_time) < self.double_press_interval:
                # Double press
                self.right_stick_mode_index = (self.right_stick_mode_index + 2) % len(self.right_stick_modes)
            else:
                # Single press
                self.right_stick_mode_index = (self.right_stick_mode_index + 1) % len(self.right_stick_modes)
            self.rb_last_press_time = current_time
            self.print_controls()

        # Left Bumper (LB) - button 4
        if msg.buttons[4] == 1:
            current_time = time.time()
            if (current_time - self.lb_last_press_time) < self.double_press_interval:
                # Double press
                self.left_stick_mode_index = (self.left_stick_mode_index + 2) % len(self.left_stick_modes)
            else:
                # Single press
                self.left_stick_mode_index = (self.left_stick_mode_index + 1) % len(self.left_stick_modes)
            self.lb_last_press_time = current_time
            self.print_controls()

        # --- Analog Stick Logic for Pose Update ---
        sensitivity = 0.01
        rot_sensitivity = 0.02

        # Right Stick (axes 3 for L/R, 4 for U/D)
        right_stick_lr = msg.axes[3]
        right_stick_ud = msg.axes[4]

        right_mode = self.right_stick_modes[self.right_stick_mode_index]
        if right_mode == 'xy':
            self.current_pose.x += right_stick_ud * sensitivity
            self.current_pose.y -= right_stick_lr * sensitivity
        elif right_mode == 'xz':
            self.current_pose.x += right_stick_ud * sensitivity
            self.current_pose.z += right_stick_lr * sensitivity
        elif right_mode == 'yz':
            self.current_pose.y += right_stick_ud * sensitivity
            self.current_pose.z += right_stick_lr * sensitivity

        # Left Stick (axes 0 for L/R, 1 for U/D)
        left_stick_lr = msg.axes[0]
        left_stick_ud = msg.axes[1]

        left_mode = self.left_stick_modes[self.left_stick_mode_index]
        if left_mode == 'roll_pitch':
            self.current_pose.roll += left_stick_lr * rot_sensitivity
            self.current_pose.pitch += left_stick_ud * rot_sensitivity
        elif left_mode == 'pitch_yaw':
            self.current_pose.pitch += left_stick_ud * rot_sensitivity
            self.current_pose.yaw += left_stick_lr * rot_sensitivity
        elif left_mode == 'yaw_roll':
            self.current_pose.yaw += left_stick_ud * rot_sensitivity
            self.current_pose.roll += left_stick_lr * rot_sensitivity

        # Publish the updated pose
        self.pos_cmd_publisher.publish(self.current_pose)

def main(args=None):
    rclpy.init(args=args)
    node = XboxToPosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
