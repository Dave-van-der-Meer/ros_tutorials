from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_turtlesim',
            executable='my_first_node',
            name='my_first_program',
            output='screen',
        )
    ])
