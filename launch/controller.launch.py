from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='controller_to_ros_msg',
            executable='xbox_to_pos_node',
            name='controller_to_pos_cmd',
            output='screen'
        )
    ])
