from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtle_control',
            executable='figure8_driver',
            name='figure8_driver',
            output='screen'
        )
    ])
