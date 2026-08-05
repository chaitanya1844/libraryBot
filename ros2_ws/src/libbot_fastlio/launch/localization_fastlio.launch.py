"""Nav2's localization source: FAST-LIO2 (3D lidar-inertial odometry)
instead of AMCL. Replaces `localization.launch.py use_amcl:=true` --
don't run both at once, they'd fight over publishing `map -> odom`.

Still starts `map_server` (Nav2's costmaps need the static 2D map
regardless of what provides localization) and a lifecycle manager for
it, plus the same 3D lidar Velodyne-format adapter + FAST-LIO2
`fastlio_mapping` node as `localization_3d.launch.py`.

Real relocalization against `pcd_map_path` (a PCD saved by a previous
fastlio_mapping run -- see pcd_to_2d_map.py's docstring for how that PCD
also becomes the `map` arg's 2D yaml/pgm) is libbot_fastlio's
`global_localization.py`: a ROS2 port of HViktorTsoi/FAST_LIO_LOCALIZATION,
proper coarse-to-fine ICP against the saved map, seeded by RViz's
"2D Pose Estimate" (same as AMCL needs) and continuously re-registering
afterward. `fastlio_map_odom_bridge` (use_external_localization:=true)
just relays its /map_to_odom output through this project's own
map -> odom math (composing through the fixed base<->body offset and
this project's EKF-based odom frame) -- see both nodes' own docstrings
for why a single local ICP pass here wasn't reliable enough on its own.

Requires the separate ~/fastlio_ws overlay sourced first, same as
localization_3d.launch.py:

    source /opt/ros/humble/setup.bash
    source ~/fastlio_ws/install/setup.bash
    source ~/libraryBot/ros2_ws/install/setup.bash
    ros2 launch libbot_fastlio localization_fastlio.launch.py \\
        map:=/path/to/name.yaml pcd_map_path:=/path/to/name.pcd
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    pcd_map_path = LaunchConfiguration('pcd_map_path')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('libbot_fastlio'), 'maps', 'library.yaml')),
        DeclareLaunchArgument('pcd_map_path', default_value=os.path.expanduser(
            '~/libraryBot/ros2_ws/src/libbot_fastlio/pcd_maps/latest.pcd')),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        ),
        TimerAction(period=2.0, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server'],
                }],
            ),
        ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('libbot_fastlio'),
                'launch', 'localization_3d.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # map_en/pcd_save off: FAST-LIO2 has no localize-only mode
                # (always runs live SLAM), but there's no reason to have it
                # accumulate its own point cloud on top of the fixed PCD
                # we're actually localizing against -- just a "why does it
                # look like it's still mapping" distraction otherwise.
                'config_file': 'fast_lio_libbot_localize.yaml',
            }.items(),
        ),

        Node(
            package='libbot_fastlio',
            executable='global_localization.py',
            name='global_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'pcd_map_path': pcd_map_path,
                # Default (0.8) is tuned for dense real-hardware lidar
                # scans; this project's simulated 3D lidar (16 channels,
                # 5Hz) produces sparser scans that measured consistently
                # converging around 0.79-0.80 fitness -- a real, stable
                # match, just short of an arbitrary threshold that fits
                # denser data.
                'localization_threshold': 0.6,
            }],
        ),

        Node(
            package='libbot_fastlio',
            executable='fastlio_map_odom_bridge.py',
            name='fastlio_map_odom_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'use_external_localization': True}],
        ),

        # Same live color-painted cloud as mapping.launch.py -- FAST-LIO2
        # still publishes /cloud_registered during localization (it's
        # just localizing against a fixed prior map, not saving a new
        # one), so this keeps building a colored view of wherever the
        # robot drives during nav/demo too.
        Node(
            package='libbot_perception',
            executable='lidar_camera_colorizer.py',
            name='lidar_camera_colorizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Same /path trimming as mapping.launch.py -- FAST-LIO2 still
        # publishes /path during localization too.
        Node(
            package='libbot_perception',
            executable='path_trimmer_node.py',
            name='path_trimmer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Static visualization of the actual PCD being localized against
        # (RViz's FastLioCloud display) -- with map_en off (see above)
        # there's otherwise nothing at all shown in 3D for "the map"
        # during localization. Republishes periodically (interval, not a
        # single one-shot publish) since it's Volatile QoS: a late
        # subscriber (RViz reconnecting, etc.) would otherwise never see
        # a one-shot publish that already happened.
        Node(
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='pcd_to_pointcloud',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'file_name': pcd_map_path,
                'tf_frame': 'map',
                'interval': 5.0,
            }],
            remappings=[('cloud_pcd', '/Laser_map')],
        ),
    ])
