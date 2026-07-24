"""Flattens the 3D lidar point cloud into a 2D /libbot/scan for SLAM/Nav2.

    ros2 launch libbot_perception lidar_processing.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('libbot_perception')
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(
            pkg_share, 'config', 'pointcloud_to_laserscan.yaml')),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[config],
            remappings=[
                ('cloud_in', '/libbot/lidar_3d/points'),
                # global /scan, not /libbot/scan: slam_toolbox publishes its
                # output map on a hardcoded absolute "/map" regardless of
                # node namespace, so the whole nav stack (AMCL, slam_toolbox,
                # Nav2) is kept un-namespaced here to match that convention
                # and avoid partially-namespaced topic confusion.
                ('scan', '/scan'),
            ],
        ),
    ])
