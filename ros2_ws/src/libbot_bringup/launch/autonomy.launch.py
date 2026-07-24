"""Second-stage bringup: Nav2 itself (planner/controller/behaviors). Run
this AFTER simulation.launch.py (mode:=navigation) is up and AMCL has
localized the robot -- Nav2 needs map -> odom -> base_footprint already
flowing.

    ros2 launch libbot_bringup autonomy.launch.py

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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('libbot_navigation'),
                'launch', 'navigation.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
