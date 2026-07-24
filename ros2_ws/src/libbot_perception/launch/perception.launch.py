"""RFID reader + depth-camera shelf-slot detector.

    ros2 launch libbot_perception perception.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sorting_share = get_package_share_directory('libbot_sorting')

    book_catalog_path = LaunchConfiguration('book_catalog_path')
    shelf_map_path = LaunchConfiguration('shelf_map_path')
    read_range = LaunchConfiguration('read_range')

    return LaunchDescription([
        DeclareLaunchArgument('book_catalog_path', default_value=os.path.join(
            sorting_share, 'config', 'book_catalog.yaml')),
        DeclareLaunchArgument('shelf_map_path', default_value=os.path.join(
            sorting_share, 'config', 'shelf_map.yaml')),
        DeclareLaunchArgument('read_range', default_value='0.55',
                               description='Meters: how close the robot must be to a book prop to "read" its tag'),

        Node(
            package='libbot_perception',
            executable='rfid_reader_node.py',
            name='rfid_reader_node',
            output='screen',
            parameters=[{
                'book_catalog_path': book_catalog_path,
                'shelf_map_path': shelf_map_path,
                'read_range': read_range,
            }],
        ),
        Node(
            package='libbot_perception',
            executable='shelf_slot_detector_node.py',
            name='shelf_slot_detector_node',
            output='screen',
        ),
    ])
