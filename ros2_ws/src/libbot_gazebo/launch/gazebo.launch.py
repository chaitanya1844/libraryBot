"""Starts Gazebo Classic (gzserver [+ gzclient]) loaded with the library world.

    ros2 launch libbot_gazebo gazebo.launch.py
    ros2 launch libbot_gazebo gazebo.launch.py headless:=true
    ros2 launch libbot_gazebo gazebo.launch.py world:=empty.world
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('libbot_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    verbose = LaunchConfiguration('verbose')

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = os.path.join(pkg_gazebo, 'models')
    new_model_path = model_path + (os.pathsep + existing_model_path if existing_model_path else '')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_gazebo, 'worlds', 'library.world'),
                               description='Absolute path to a .world file'),
        DeclareLaunchArgument('headless', default_value='false',
                               description='Run gzserver only, without the Gazebo GUI client'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world,
                'verbose': verbose,
                'pause': 'false',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            condition=UnlessCondition(headless),
        ),
    ])
