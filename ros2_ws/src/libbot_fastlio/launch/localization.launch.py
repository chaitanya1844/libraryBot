"""EKF (always) + optionally full localization against a saved map.

TF ownership:
    robot_state_publisher : base_footprint -> base_link -> sensor frames
    EKF (this launch)      : odom -> base_footprint
    FAST-LIO2 (if enabled)  : map -> odom, via fastlio_map_odom_bridge
                              (see localization_fastlio.launch.py)
    FAST-LIO2 (mapping)    : map -> camera_init -> body, no map -> odom
                              at all (see libbot_fastlio/mapping.launch.py) --
                              mutually exclusive with the above regardless

`use_amcl` is a legacy name kept for backward compatibility with
existing callers (simulation.launch.py, switch_mode.sh) -- it no longer
starts nav2_amcl. AMCL was replaced with FAST-LIO2 (3D lidar-inertial
odometry) as the actual localization source; see
localization_fastlio.launch.py and the README's "3D localization"
section for why (redundant to run two localization sources at once, and
this is what the 3D lidar is actually for).

    ros2 launch libbot_fastlio localization.launch.py
    ros2 launch libbot_fastlio localization.launch.py use_amcl:=true map:=/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include_localization_fastlio(context, use_sim_time, use_amcl, map_yaml, pkg_share):
    # Named PCD matching the map's own basename (pcd_to_2d_map.py copies the
    # source PCD there under that name at save time) -- NOT always
    # pcd_maps/latest.pcd regardless of which map was actually selected: that
    # earlier behavior silently paired a newer/different mapping session's
    # point cloud with an older selected map (see switch_mode.sh, which has
    # the same fallback logic for its own launch path). Falls back to
    # latest.pcd only if no matching named PCD exists yet.
    # Source-tree path (not pkg_share/install) -- pcd_maps are large,
    # machine/run-specific files maintained in place, same convention
    # localization_fastlio.launch.py and switch_mode.sh already use.
    pcd_maps_dir = os.path.expanduser(
        '~/libraryBot/ros2_ws/src/libbot_fastlio/pcd_maps')
    map_yaml_path = map_yaml.perform(context)
    map_basename = os.path.splitext(os.path.basename(map_yaml_path))[0]
    named_pcd_path = os.path.join(pcd_maps_dir, f'{map_basename}.pcd')
    if os.path.isfile(named_pcd_path):
        pcd_map_path = named_pcd_path
    else:
        pcd_map_path = os.path.join(pcd_maps_dir, 'latest.pcd')

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_share, 'launch', 'localization_fastlio.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'pcd_map_path': pcd_map_path,
        }.items(),
        condition=IfCondition(use_amcl),
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('libbot_fastlio')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amcl = LaunchConfiguration('use_amcl')
    map_yaml = LaunchConfiguration('map')
    ekf_config = LaunchConfiguration('ekf_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_amcl', default_value='false',
                               description='Localize against a saved map with FAST-LIO2. '
                                            'Leave false while mapping (libbot_fastlio '
                                            'mapping.launch.py).'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('libbot_fastlio'), 'maps', 'library.yaml')),
        DeclareLaunchArgument('ekf_config', default_value=os.path.join(pkg_share, 'config', 'ekf.yaml')),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        ),

        OpaqueFunction(function=lambda context: _include_localization_fastlio(
            context, use_sim_time, use_amcl, map_yaml, pkg_share)),
    ])
