"""Xbox 360 controller teleop for manual driving (mapping/testing), remapped
onto the mecanum controller's command topic. The controller connects via its
USB wireless dongle and shows up as /dev/input/js0.

    ros2 launch libbot_control teleop_joy.launch.py

Controls (see config/xbox_holonomic.yaml):
  Hold LB (left bumper)         -- deadman switch, robot only moves while held
  Left stick up/down            -- forward / backward
  Left stick left/right         -- strafe left / right (holonomic)
  Right stick left/right        -- rotate in place
  Hold LB + RB (right bumper)   -- turbo (faster) speed scale

If the controller isn't found, check `ls /dev/input/js*` and
`ls -l /dev/input/by-id/` -- it should show a *-joystick symlink to js0.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_config = os.path.join(
        get_package_share_directory('libbot_control'), 'config', 'xbox_holonomic.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config],
            remappings=[('/cmd_vel', '/libbot/mecanum_drive_controller/reference_unstamped')],
        ),
    ])
