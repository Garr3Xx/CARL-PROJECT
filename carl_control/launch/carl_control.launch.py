from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carl_control',
            executable='carl_control_node',
            name='carl_control_node',
            output='screen'
        )
    ])
