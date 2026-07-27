"""Core bringup: Gazebo + robot + control + lidar processing + EKF +
Xbox teleop + control panel + RViz. Does NOT start SLAM or AMCL by
default -- drive the
robot around with the RViz-embedded control panel's "Start Mapping" /
"Start Localization" / "Start Navigation" buttons to move through that
workflow interactively (see README "Map, localize, navigate workflow").

    ros2 launch libbot_bringup simulation.launch.py
    ros2 launch libbot_bringup simulation.launch.py headless:=true
    ros2 launch libbot_bringup simulation.launch.py control_panel:=false rviz:=false

Launch args `mode` (mapping/navigation/none, default none) and `map` are
still available for scripting a specific stage directly instead of using
the panel buttons, e.g.:

    ros2 launch libbot_bringup simulation.launch.py mode:=mapping
    ros2 launch libbot_bringup simulation.launch.py mode:=navigation map:=/path/to/map.yaml

`mode:=navigation` here only starts AMCL (localization) -- Nav2 itself
(planning/control) is the separate `autonomy.launch.py`, started once
localization is confirmed working
(Nav2 nested inside this same launch file hit a `launch_ros`/
`IncludeLaunchDescription` parameter-loading quirk that reliably drops
DWB's critics list when started this way -- see README "Two-stage
bringup" for the full story):

    ros2 launch libbot_bringup autonomy.launch.py

See the README for the full list of tunable launch arguments and where
their underlying configs live.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _include(pkg, launch_file, launch_arguments=None, condition=None, delay=0.0):
    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )
    if delay <= 0.0:
        return action
    return TimerAction(period=delay, actions=[action])


def generate_launch_description():
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    control_panel = LaunchConfiguration('control_panel')
    rviz = LaunchConfiguration('rviz')
    teleop_joy = LaunchConfiguration('teleop_joy')

    is_mapping = PythonExpression(["'", mode, "' == 'mapping'"])
    is_navigation = PythonExpression(["'", mode, "' == 'navigation'"])

    pkg_navigation = get_package_share_directory('libbot_navigation')
    rviz_config = os.path.join(pkg_navigation, 'rviz', 'libbot_nav.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(
            get_package_share_directory('libbot_gazebo'), 'worlds', 'library.world')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mode', default_value='none',
                               description="'mapping', 'navigation', or 'none' "
                                            "(use the panel's Mode buttons instead)"),
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('libbot_mapping'), 'maps', 'library.yaml')),
        DeclareLaunchArgument('x', default_value='1.0'),
        DeclareLaunchArgument('y', default_value='3.3'),
        # Flipped 180deg from the original 1.5708 spawn yaw per user request.
        DeclareLaunchArgument('yaw', default_value='-1.5708'),
        # The RViz-embedded "libraryBot Controls" panel (libbot_rviz_plugins)
        # is the primary control surface and is always part of the rviz
        # config below. This standalone PyQt5 window is a legacy/headless
        # fallback -- off by default so you don't get two copies of the
        # same controls in two separate windows.
        DeclareLaunchArgument('control_panel', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('teleop_joy', default_value='true',
                               description='Start Xbox controller teleop automatically. '
                                            'Harmless with no controller plugged in -- '
                                            'joy_node just logs that it found no device.'),

        # 1. Gazebo world
        _include('libbot_gazebo', 'gazebo.launch.py', {
            'world': world, 'headless': headless, 'use_sim_time': use_sim_time,
        }),

        # 2. Spawn the robot (small delay so gzserver is ready for spawn_entity)
        _include('libbot_gazebo', 'spawn_robot.launch.py', {
            'x': x, 'y': y, 'yaw': yaw, 'use_sim_time': use_sim_time,
        }, delay=5.0),

        # 3. Activate ros2_control controllers
        _include('libbot_control', 'control.launch.py', {}, delay=9.0),

        # 4. Lidar -> 2D scan for SLAM/Nav2
        _include('libbot_perception', 'lidar_processing.launch.py', {}, delay=9.0),

        # Breadcrumb trail of where the robot has actually been, for RViz
        # (alongside Nav2's GlobalPlan/LocalPlan, which only show where
        # it's *going*). Harmless to start early -- it just waits for
        # /odometry/filtered to exist.
        TimerAction(period=12.0, actions=[Node(
            package='libbot_perception',
            executable='trail_publisher_node.py',
            name='trail_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )]),

        # 5. Localization: EKF always; SLAM (mapping) or AMCL (navigation) exclusively
        _include('libbot_mapping', 'mapping.launch.py', {'use_sim_time': use_sim_time},
                 condition=IfCondition(is_mapping), delay=11.0),
        _include('libbot_localization', 'localization.launch.py', {
            'use_sim_time': use_sim_time,
            'use_amcl': is_navigation,
            'map': map_yaml,
        }, delay=11.0),

        # 6. Xbox controller teleop -- manual driving for mapping runs
        _include('libbot_control', 'teleop_joy.launch.py', {},
                 condition=IfCondition(teleop_joy), delay=6.0),

        # 7. Control panel + RViz
        _include('libbot_control_panel', 'control_panel.launch.py', {},
                 condition=IfCondition(control_panel), delay=6.0),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
