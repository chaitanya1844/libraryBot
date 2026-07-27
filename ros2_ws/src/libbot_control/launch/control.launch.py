"""Activates the ros2_control controllers loaded by gazebo_ros2_control.

    ros2 launch libbot_control control.launch.py

Canonical velocity command topic for the whole stack:
    /libbot/mecanum_drive_controller/reference   (geometry_msgs/Twist)

Nav2's controller_server, the control panel, and teleop all publish here.
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/libbot/controller_manager'],
        output='screen',
    )

    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller',
                   '--controller-manager', '/libbot/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/libbot/controller_manager'],
        output='screen',
    )

    # Give gazebo_ros2_control a moment to bring up the controller_manager
    # before the spawners try to call its services.
    delayed_mecanum_spawner = TimerAction(period=3.0, actions=[mecanum_drive_controller_spawner])
    delayed_arm_spawner = TimerAction(period=3.0, actions=[arm_controller_spawner])

    # Send the arm to its stowed pose once arm_controller is up, so it
    # never sits at its spawn/zero pose -- with the arm mounted on the
    # chassis side, all-joints-zero extends the whole arm straight out
    # (~0.7m), which gets in the way navigating tight corners/aisles
    # (mapping runs especially) until something explicitly commands it
    # elsewhere, e.g. libbot_manipulation's task manager.
    arm_stow_node = Node(
        package='libbot_control',
        executable='arm_stow_node.py',
        name='arm_stow_node',
        output='screen',
    )
    delayed_arm_stow = TimerAction(period=6.0, actions=[arm_stow_node])

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        delayed_mecanum_spawner,
        delayed_arm_spawner,
        delayed_arm_stow,
    ])
