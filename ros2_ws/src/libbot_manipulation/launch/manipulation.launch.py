"""Starts the book-collection task manager.

    ros2 launch libbot_manipulation manipulation.launch.py

Run this after Nav2 (autonomy.launch.py) is up and localized -- the task
manager is a NavigateToPose client and does nothing useful without a
running planner/controller to drive to.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='libbot_manipulation',
            executable='task_manager_node.py',
            name='libbot_task_manager',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
