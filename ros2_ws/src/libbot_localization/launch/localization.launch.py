"""EKF (always) + optionally full localization against a saved map.

TF ownership:
    robot_state_publisher : base_footprint -> base_link -> sensor frames
    EKF (this launch)      : odom -> base_footprint
    FAST-LIO2 (if enabled)  : map -> odom, via fastlio_map_odom_bridge
                              (see localization_fastlio.launch.py)
    FAST-LIO2 (mapping)    : map -> camera_init -> body, no map -> odom
                              at all (see libbot_mapping/mapping.launch.py) --
                              mutually exclusive with the above regardless

`use_amcl` is a legacy name kept for backward compatibility with
existing callers (simulation.launch.py, switch_mode.sh) -- it no longer
starts nav2_amcl. AMCL was replaced with FAST-LIO2 (3D lidar-inertial
odometry) as the actual localization source; see
localization_fastlio.launch.py and the README's "3D localization"
section for why (redundant to run two localization sources at once, and
this is what the 3D lidar is actually for).

    ros2 launch libbot_localization localization.launch.py
    ros2 launch libbot_localization localization.launch.py use_amcl:=true map:=/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('libbot_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amcl = LaunchConfiguration('use_amcl')
    map_yaml = LaunchConfiguration('map')
    ekf_config = LaunchConfiguration('ekf_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_amcl', default_value='false',
                               description='Localize against a saved map with FAST-LIO2. '
                                            'Leave false while mapping (libbot_mapping '
                                            'mapping.launch.py).'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('libbot_mapping'), 'maps', 'library.yaml')),
        DeclareLaunchArgument('ekf_config', default_value=os.path.join(pkg_share, 'config', 'ekf.yaml')),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_share, 'launch', 'localization_fastlio.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'map': map_yaml}.items(),
            condition=IfCondition(use_amcl),
        ),
    ])
