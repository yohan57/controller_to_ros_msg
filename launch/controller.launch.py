from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}] # Adjust if your controller is on a different device
        ),
        Node(
            package='controller_to_ros_msg',
            executable='xbox_to_pos_node',
            name='controller_to_pos_cmd',
            output='screen',
        ),
    ])
