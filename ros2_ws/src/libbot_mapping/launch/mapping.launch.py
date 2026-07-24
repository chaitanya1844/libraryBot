"""SLAM Toolbox online-async mapping. Run this INSTEAD of AMCL localization
(both publish map -> odom; running both is a TF conflict).

    ros2 launch libbot_mapping mapping.launch.py

Drive the robot around (teleop or control panel), then save the map:

    ros2 run nav2_map_server map_saver_cli -f /home/sychic/libraryBot/ros2_ws/src/libbot_mapping/maps/library
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('libbot_mapping')
    slam_config = LaunchConfiguration('slam_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_config', default_value=os.path.join(
            pkg_share, 'config', 'slam_toolbox_online_async.yaml')),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': use_sim_time}],
        ),
    ])
