"""Launch file for dante_bringup example node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dante_bringup',
            executable='dante_node',
            name='dante_node'
        )
    ])
