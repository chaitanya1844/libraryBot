"""Standalone GUI control panel.

    ros2 launch libbot_control_panel control_panel.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='libbot_control_panel',
            executable='control_panel_node.py',
            name='libbot_control_panel',
            output='screen',
        ),
    ])
