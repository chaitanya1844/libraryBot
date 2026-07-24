"""Starts the RFID-to-shelf sorting task manager.

Prerequisites: Gazebo + robot spawned, control active, localization
(AMCL against a saved map) and navigation both running, and
libbot_perception's perception.launch.py (RFID reader + shelf-slot
detector) running.

    ros2 launch libbot_sorting sorting.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('libbot_sorting')
    shelf_map_path = LaunchConfiguration('shelf_map_path')
    auto_start = LaunchConfiguration('auto_start')

    return LaunchDescription([
        DeclareLaunchArgument('shelf_map_path', default_value=os.path.join(
            pkg_share, 'config', 'shelf_map.yaml')),
        DeclareLaunchArgument('auto_start', default_value='true',
                               description='If false, the task manager queues jobs but '
                                            'waits for /libbot/sorting/enable before acting'),

        Node(
            package='libbot_sorting',
            executable='task_manager_node.py',
            name='sorting_task_manager',
            output='screen',
            parameters=[{
                'shelf_map_path': shelf_map_path,
                'auto_start': auto_start,
            }],
        ),
    ])
