"""Nav2 planning/control stack (controller, planner, behaviors, bt_navigator,
waypoint_follower) -- NOT AMCL/map_server, which live in
libbot_localization/launch/localization.launch.py so localization can be
restarted independently of planning.

    ros2 launch libbot_navigation navigation.launch.py

Prerequisites already running: libbot_gazebo spawn_robot, libbot_control
control, libbot_perception lidar_processing, and
libbot_localization localization.launch.py use_amcl:=true map:=<path>.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('libbot_navigation')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Nav2's controller_server publishes plain Twist on "cmd_vel" by
    # default; remap straight onto the mecanum controller's command topic,
    # the same canonical one teleop and the control panel use.
    cmd_vel_remap = ('cmd_vel', '/libbot/mecanum_drive_controller/reference_unstamped')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            pkg_share, 'config', 'nav2_params.yaml')),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[cmd_vel_remap],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file],
        ),
        # Delayed relative to its sibling servers above: lifecycle_manager
        # calling configure() before controller_server (in particular) has
        # finished constructing its own lifecycle services is a real race
        # under load -- it fails fast with "No critics defined for
        # FollowPath" rather than retrying, because DWB had not finished
        # loading by the time configure() ran, not because the param file
        # was actually missing anything. A few seconds' head start avoids it.
        TimerAction(period=4.0, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes,
                }],
            ),
        ]),
    ])
