"""3D localization/mapping using the robot's own 3D lidar + IMU, via
FAST-LIO2 (tightly-coupled LiDAR-inertial odometry, iterated Kalman
filter + incremental k-d tree -- no factor-graph backend, chosen over
rtabmap specifically for being light enough to actually run on a Jetson
Nano Super). This is what /libbot/lidar_3d/points and /libbot/imu/data_raw
are for -- the rest of the stack only ever flattens the lidar cloud to 2D
via pointcloud_to_laserscan.

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
    ros2 launch libbot_localization localization_3d.launch.py
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

    fast_lio_share = get_package_share_directory('fast_lio')
    libbot_localization_share = get_package_share_directory('libbot_localization')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false',
                               description='Launch FAST-LIO2\'s own RViz config'),

        # Gazebo's ray sensor has no per-point ring/time capture (the
        # whole cloud arrives at one simulation instant) -- FAST-LIO's
        # Velodyne point type needs both, synthesized here from each
        # point's geometry (see the node's own docstring for why this is
        # safe to fabricate in sim).
        Node(
            package='libbot_perception',
            executable='lidar_3d_velodyne_adapter.py',
            name='lidar_3d_velodyne_adapter',
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
                'config_path': os.path.join(libbot_localization_share, 'config'),
                'config_file': 'fast_lio_libbot.yaml',
                'rviz': rviz,
            }.items(),
        ),
    ])
