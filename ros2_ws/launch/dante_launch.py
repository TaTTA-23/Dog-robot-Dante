"""Launch file for dante_bringup (minimal).

This launch file is a simple example that would launch the Python node if
ROS2 is available. It is intentionally small so it can be extended later.
"""
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
