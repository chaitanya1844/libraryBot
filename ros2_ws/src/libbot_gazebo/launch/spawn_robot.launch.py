"""Publishes robot_description (with Gazebo/ros2_control tags) and spawns
the libbot model into a running Gazebo instance.

    ros2 launch libbot_gazebo spawn_robot.launch.py
    ros2 launch libbot_gazebo spawn_robot.launch.py x:=1.0 y:=3.3 yaw:=1.5708
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('libbot_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'libbot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')

    # See scripts/strip_urdf_for_gzcontrol.py: works around a
    # gazebo_ros2_control parser bug ("Couldn't parse parameter override
    # rule") triggered by xacro's comment header when it re-ingests
    # robot_description as a --param override.
    pkg_gazebo = get_package_share_directory('libbot_gazebo')
    strip_script = os.path.join(pkg_gazebo, '..', '..', 'lib', 'libbot_gazebo',
                                 'strip_urdf_for_gzcontrol.py')
    strip_script = os.path.normpath(strip_script)

    shell_cmd = 'xacro {0} use_gazebo:=true | {1}'.format(xacro_file, strip_script)
    robot_description = ParameterValue(
        Command(['bash -c "{0}"'.format(shell_cmd)]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='libbot'),
        # Default spawn pose: open floor in front of the intake / InfoDesk area.
        DeclareLaunchArgument('x', default_value='1.0'),
        DeclareLaunchArgument('y', default_value='3.3'),
        DeclareLaunchArgument('z', default_value='0.05'),
        # Flipped 180deg from the original 1.5708 spawn yaw per user request.
        DeclareLaunchArgument('yaw', default_value='-1.5708'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='libbot',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_libbot',
            output='screen',
            arguments=[
                '-topic', '/libbot/robot_description',
                '-entity', robot_name,
                '-x', x, '-y', y, '-z', z,
                '-Y', yaw,
            ],
        ),
    ])
