"""Keyboard teleop for manual driving, remapped onto the mecanum controller's
command topic. Run in its own terminal (it needs an interactive TTY):

    ros2 launch libbot_control teleop.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
            remappings=[('/cmd_vel', '/libbot/mecanum_drive_controller/reference_unstamped')],
        ),
    ])
