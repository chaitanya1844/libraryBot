"""Nav2's localization source: FAST-LIO2 (3D lidar-inertial odometry)
instead of AMCL. Replaces `localization.launch.py use_amcl:=true` --
don't run both at once, they'd fight over publishing `map -> odom`.

Still starts `map_server` (Nav2's costmaps need the static 2D map
regardless of what provides localization) and a lifecycle manager for
it, plus the same 3D lidar Velodyne-format adapter + FAST-LIO2
`fastlio_mapping` node as `localization_3d.launch.py`, plus
`fastlio_map_odom_bridge` to turn FAST-LIO2's own pose estimate into the
`map -> odom` transform Nav2 actually looks for (see that node's
docstring for the exact math and the assumption it relies on: FAST-LIO2
must initialize at the same pose the map/spawn convention already
assumes, same as AMCL's `initial_pose` in `amcl.yaml` had to).

Requires the separate ~/fastlio_ws overlay sourced first, same as
localization_3d.launch.py:

    source /opt/ros/humble/setup.bash
    source ~/fastlio_ws/install/setup.bash
    source ~/libraryBot/ros2_ws/install/setup.bash
    ros2 launch libbot_localization localization_fastlio.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('libbot_mapping'), 'maps', 'library.yaml')),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        ),
        TimerAction(period=2.0, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server'],
                }],
            ),
        ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('libbot_localization'),
                'launch', 'localization_3d.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='libbot_localization',
            executable='fastlio_map_odom_bridge.py',
            name='fastlio_map_odom_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
