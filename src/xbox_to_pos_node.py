import rclpy
from rclpy.node import Node
from piper_msgs.msg import PosCmd
import time
import inputs
import threading

class XboxToPosNode(Node):
    def __init__(self):
        super().__init__('xbox_to_pos_node')

        # Publisher for piper_ros
        self.pos_cmd_publisher = self.create_publisher(PosCmd, '/pos_cmd', 10)

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

        self.axes = {'ABS_X': 0, 'ABS_Y': 0, 'ABS_RX': 0, 'ABS_RY': 0}
        self.buttons = {'BTN_TR': 0, 'BTN_TL': 0, 'BTN_SOUTH': 0, 'BTN_EAST': 0, 'BTN_NORTH': 0, 'BTN_WEST': 0}

        self.get_logger().info('Xbox to Piper PosCmd node started.')
        self.print_controls()

        self.controller_thread = threading.Thread(target=self.process_controller_events)
        self.controller_thread.daemon = True
        self.controller_thread.start()

        self.publisher_timer = self.create_timer(0.1, self.publish_pose)


    def print_controls(self):
        self.get_logger().info("--- Controls ---")
        self.get_logger().info(f"Right Stick Mode: {self.right_stick_modes[self.right_stick_mode_index].upper()} -> Left/Right: {self.right_stick_modes[self.right_stick_mode_index][0]}, Up/Down: {self.right_stick_modes[self.right_stick_mode_index][1]}")
        self.get_logger().info(f"Left Stick Mode: {self.left_stick_modes[self.left_stick_mode_index].upper()} -> Left/Right: {self.left_stick_modes[self.left_stick_mode_index].split('_')[1]}, Up/Down: {self.left_stick_modes[self.left_stick_mode_index].split('_')[0]}")
        self.get_logger().info("RB (Right Bumper): Change Right Stick Mode")
        self.get_logger().info("LB (Left Bumper): Change Left Stick Mode")
        if self.right_stick_modes[self.right_stick_mode_index] == 'xy':
            self.get_logger().info("A: Z Down, Y: Z Up")
            self.get_logger().info("X: Gripper Close, B: Gripper Open")
        self.get_logger().info("----------------")

    def process_controller_events(self):
        try:
            gamepad = inputs.devices.gamepads[0]
        except IndexError:
            self.get_logger().error("No gamepad found.")
            return

        while rclpy.ok():
            try:
                events = gamepad.read()
            except EOFError:
                self.get_logger().error("Gamepad disconnected.")
                break
            for event in events:
                if event.ev_type == 'Key':
                    self.handle_button_press(event)
                elif event.ev_type == 'Absolute':
                    self.handle_stick_move(event)

    def handle_button_press(self, event):
        if event.code in self.buttons:
            self.buttons[event.code] = event.state

        if event.code == 'BTN_TR': # Right Bumper (RB)
            if event.state == 1: # Button pressed
                current_time = time.time()
                if (current_time - self.rb_last_press_time) < self.double_press_interval:
                    self.right_stick_mode_index = (self.right_stick_mode_index + 2) % len(self.right_stick_modes)
                else:
                    self.right_stick_mode_index = (self.right_stick_mode_index + 1) % len(self.right_stick_modes)
                self.rb_last_press_time = current_time
                self.print_controls()

        if event.code == 'BTN_TL': # Left Bumper (LB)
            if event.state == 1: # Button pressed
                current_time = time.time()
                if (current_time - self.lb_last_press_time) < self.double_press_interval:
                    self.left_stick_mode_index = (self.left_stick_mode_index + 2) % len(self.left_stick_modes)
                else:
                    self.left_stick_mode_index = (self.left_stick_mode_index + 1) % len(self.left_stick_modes)
                self.lb_last_press_time = current_time
                self.print_controls()

    def handle_stick_move(self, event):
        # Normalize axis value from -32768 to 32767 to -1.0 to 1.0
        value = event.state / 32768.0
        self.axes[event.code] = value

    def publish_pose(self):
        sensitivity = 0.01
        rot_sensitivity = 0.02
        gripper_sensitivity = 0.05

        # Right Stick
        right_stick_lr = self.axes.get('ABS_RX', 0.0)
        right_stick_ud = self.axes.get('ABS_RY', 0.0)

        right_mode = self.right_stick_modes[self.right_stick_mode_index]
        if right_mode == 'xy':
            self.current_pose.x -= right_stick_ud * sensitivity
            self.current_pose.y -= right_stick_lr * sensitivity
            
            # Z-axis control with A and Y
            if self.buttons.get('BTN_SOUTH', 0) == 1: # A button
                self.current_pose.z -= sensitivity
            if self.buttons.get('BTN_WEST', 0) == 1: # Y button
                self.current_pose.z += sensitivity

            # Gripper control with X and B
            if self.buttons.get('BTN_NORTH', 0) == 1: # X button
                self.current_pose.gripper -= gripper_sensitivity
            if self.buttons.get('BTN_EAST', 0) == 1: # B button
                self.current_pose.gripper += gripper_sensitivity
            
            # Clamp gripper value
            self.current_pose.gripper = max(0.0, min(1.0, self.current_pose.gripper))

        elif right_mode == 'xz':
            self.current_pose.x -= right_stick_ud * sensitivity
            self.current_pose.z += right_stick_lr * sensitivity
        elif right_mode == 'yz':
            self.current_pose.y -= right_stick_ud * sensitivity
            self.current_pose.z += right_stick_lr * sensitivity

        # Left Stick
        left_stick_lr = self.axes.get('ABS_X', 0.0)
        left_stick_ud = self.axes.get('ABS_Y', 0.0)

        left_mode = self.left_stick_modes[self.left_stick_mode_index]
        if left_mode == 'roll_pitch':
            self.current_pose.roll += left_stick_lr * rot_sensitivity
            self.current_pose.pitch -= left_stick_ud * rot_sensitivity
        elif left_mode == 'pitch_yaw':
            self.current_pose.pitch -= left_stick_ud * rot_sensitivity
            self.current_pose.yaw += left_stick_lr * rot_sensitivity
        elif left_mode == 'yaw_roll':
            self.current_pose.yaw -= left_stick_ud * rot_sensitivity
            self.current_pose.roll += left_stick_lr * rot_sensitivity

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