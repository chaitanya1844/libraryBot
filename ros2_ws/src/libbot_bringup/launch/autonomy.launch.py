"""Second-stage bringup: Nav2 + RFID/depth-camera perception + the sorting
task manager. Run this AFTER simulation.launch.py (mode:=navigation) is up
and AMCL has localized the robot -- Nav2 needs map -> odom -> base_footprint
already flowing.

    ros2 launch libbot_bringup autonomy.launch.py
    ros2 launch libbot_bringup autonomy.launch.py sorting_enabled:=false
    ros2 launch libbot_bringup autonomy.launch.py read_range:=0.7

This is a separate command (not folded into simulation.launch.py) because
nesting Nav2's controller_server inside another launch file's
TimerAction/IncludeLaunchDescription chain was observed to intermittently
drop DWB's `critics` parameter list ("No critics defined for FollowPath"),
while launching it directly like this has been reliable in testing. See
README "Two-stage bringup".
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(pkg, launch_file, launch_arguments=None, delay=0.0):
    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
    )
    if delay <= 0.0:
        return action
    return TimerAction(period=delay, actions=[action])


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    sorting_enabled = LaunchConfiguration('sorting_enabled')
    read_range = LaunchConfiguration('read_range')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sorting_enabled', default_value='true'),
        DeclareLaunchArgument('read_range', default_value='0.55'),

        _include('libbot_navigation', 'navigation.launch.py', {'use_sim_time': use_sim_time}),
        _include('libbot_perception', 'perception.launch.py', {'read_range': read_range}, delay=2.0),
        _include('libbot_sorting', 'sorting.launch.py', {'auto_start': sorting_enabled}, delay=4.0),
    ])
