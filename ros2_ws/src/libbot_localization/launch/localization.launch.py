"""EKF (always) + optionally AMCL against a saved map.

TF ownership:
    robot_state_publisher : base_footprint -> base_link -> sensor frames
    EKF (this launch)      : odom -> base_footprint
    AMCL (if enabled)      : map -> odom
    slam_toolbox (mapping) : map -> odom   (mutually exclusive with AMCL)

    ros2 launch libbot_localization localization.launch.py
    ros2 launch libbot_localization localization.launch.py use_amcl:=true map:=/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('libbot_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amcl = LaunchConfiguration('use_amcl')
    map_yaml = LaunchConfiguration('map')
    ekf_config = LaunchConfiguration('ekf_config')
    amcl_config = LaunchConfiguration('amcl_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_amcl', default_value='false',
                               description='Localize against a saved map with AMCL. '
                                            'Leave false while mapping with slam_toolbox.'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('libbot_mapping'), 'maps', 'library.yaml')),
        DeclareLaunchArgument('ekf_config', default_value=os.path.join(pkg_share, 'config', 'ekf.yaml')),
        DeclareLaunchArgument('amcl_config', default_value=os.path.join(pkg_share, 'config', 'amcl.yaml')),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
            condition=IfCondition(use_amcl),
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config, {'use_sim_time': use_sim_time}],
            condition=IfCondition(use_amcl),
        ),
        # Delayed relative to map_server/amcl above: calling configure()
        # before they've finished constructing their own lifecycle
        # services is a real race under system load -- lifecycle_manager
        # fails fast rather than retrying (see libbot_navigation's
        # navigation.launch.py for the same issue and README "Two-stage
        # bringup" / "Lifecycle manager races" for the full story).
        TimerAction(period=4.0, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                }],
                condition=IfCondition(use_amcl),
            ),
        ]),
    ])
