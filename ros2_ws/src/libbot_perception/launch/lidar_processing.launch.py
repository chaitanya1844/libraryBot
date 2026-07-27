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

        # Drops points that are actually the robot's own stowed arm
        # (mounted close enough to the lidar that no stow pose puts it
        # entirely outside the sensor's 360-degree ring) before anything
        # downstream sees them -- see the node's own docstring.
        Node(
            package='libbot_perception',
            executable='lidar_3d_self_filter.py',
            name='lidar_3d_self_filter',
            output='screen',
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[config],
            remappings=[
                ('cloud_in', '/libbot/lidar_3d/points_filtered'),
                # global /scan, not /libbot/scan: map_server/Nav2 publish
                # and consume "/map" as a hardcoded absolute topic
                # regardless of node namespace, so the whole nav stack is
                # kept un-namespaced here to match that convention and
                # avoid partially-namespaced topic confusion.
                ('scan', '/scan'),
            ],
        ),
    ])
