"""3D localization/mapping using the robot's own 3D lidar + IMU, via
FAST-LIO2 (tightly-coupled LiDAR-inertial odometry, iterated Kalman
filter + incremental k-d tree -- no factor-graph backend, chosen over
rtabmap specifically for being light enough to actually run on a Jetson
Nano Super). This is what /libbot/lidar_3d/points (repackaged as
/livox/lidar by lidar_3d_livox_adapter.py) and /livox/imu are for -- the
rest of the stack only ever flattens the lidar cloud to 2D via
pointcloud_to_laserscan.

FAST-LIO2 owns its own odometry (published as /Odometry, frame "camera_init"
-> "body"), independent of the EKF's "odom" used by the 2D AMCL/Nav2 stack,
so this can run alongside the 2D stack without TF conflicts, but is a
separate 3D-only localization/mapping demo, not merged into Nav2's costmaps.

Assumes simulation.launch.py (or at least libbot_perception's
lidar_processing.launch.py within it) is already running --
lidar_3d_self_filter, which strips the robot's own stowed arm out of the
lidar cloud, lives there since it's needed unconditionally (2D scan
matching needs it too), not duplicated here.

Requires the separate FAST_LIO (ROS2 branch) + livox_ros_driver2 overlay
workspace to be built and sourced first (see repo README "3D localization"
section for the from-source build steps -- these packages depend on
Livox-SDK2 for message types even though this project has no real Livox
hardware, and aren't available via apt), e.g.:

    source /opt/ros/humble/setup.bash
    source ~/fastlio_ws/install/setup.bash
    source ~/libraryBot/ros2_ws/install/setup.bash
    ros2 launch libbot_fastlio localization_3d.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    config_file = LaunchConfiguration('config_file')

    fast_lio_share = get_package_share_directory('fast_lio')
    libbot_fastlio_share = get_package_share_directory('libbot_fastlio')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false',
                               description='Launch FAST-LIO2\'s own RViz config'),
        DeclareLaunchArgument('config_file', default_value='fast_lio_libbot.yaml',
                               description='fast_lio_libbot_localize.yaml (map_en/pcd_save off) '
                                            'when localizing against an already-saved PCD instead '
                                            '-- see localization_fastlio.launch.py.'),

        # Gazebo's ray sensor has no per-point line/time capture (the
        # whole cloud arrives at one simulation instant), but the
        # livox_ros_driver2/msg/CustomMsg format FAST-LIO's Livox point
        # type needs both. `line` is bucketed from each point's vertical
        # angle; `offset_time` is left at zero rather than faked, since
        # FAST-LIO2 de-skews with it and there is no real skew here (see
        # the node's own docstring -- a correctness fix, measurably not
        # a drift fix). Publishes on /livox/lidar, the same topic a real
        # Mid360's driver uses.
        Node(
            package='libbot_perception',
            executable='lidar_3d_livox_adapter.py',
            name='lidar_3d_livox_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/libbot/lidar_3d/points_filtered',
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'config_path': os.path.join(libbot_fastlio_share, 'config'),
                'config_file': config_file,
                'rviz': rviz,
            }.items(),
        ),
    ])
