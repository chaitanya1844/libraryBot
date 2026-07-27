"""3D mapping via FAST-LIO2 (see libbot_localization/launch/localization_3d.launch.py
for the actual nodes -- this just includes it under the mapping package's
own launch name, since the control panel's "Start Mapping" button and
switch_mode.sh both look for `libbot_mapping mapping.launch.py`).

slam_toolbox (2D lidar SLAM) was the original mapping source before
FAST-LIO2 replaced AMCL as the localization source too -- running both
would fight over map -> odom, and the 2D map is now *derived from* the
3D one (pcd_to_2d_map.py) rather than built independently, so this no
longer starts slam_toolbox_online_async.

Workflow:

    ros2 launch libbot_mapping mapping.launch.py

Drive the robot around (teleop or control panel) to build up the 3D
map, then save it -- FAST-LIO2's own `/map_save` service, NOT Ctrl-C:
shutdown-time saving in this ROS2 port only writes a hardcoded path
inside the source tree (`~/fastlio_ws/src/FAST_LIO/PCD/scans.pcd`), not
the `map_file_path` configured in fast_lio_libbot.yaml, and isn't
guaranteed to run at all under `ros2 launch`'s process-group SIGINT.
(Also needs `publish.map_en: true` in that yaml -- the service saves
pcl_wait_pub, which nothing ever populates otherwise, and hits an
unhandled PCL exception that crashes the node if it's empty.)

    ros2 service call /map_save std_srvs/srv/Trigger

That writes the PCD to `map_file_path` (fast_lio_libbot.yaml, currently
libbot_mapping/pcd_maps/latest.pcd). Then convert it to the 2D
occupancy grid Nav2's costmaps need:

    ros2 run libbot_mapping pcd_to_2d_map.py /path/to/latest.pcd my_map_name

Then localize against both with:

    ros2 launch libbot_localization localization_fastlio.launch.py \\
        map:=/path/to/my_map_name.yaml pcd_map_path:=/path/to/latest.pcd

RViz's Fixed Frame is "map" (so it still works once you switch to
localization, where fastlio_map_odom_bridge solves the real map ->
camera_init offset) -- but nothing publishes "map" at all during
mapping itself, since that bridge node only runs in localization mode.
Without it, RViz can't resolve *any* transform and renders nothing.
This mapping run is what *defines* the map, so map == both odom's and
camera_init's origin here is exactly right, not just a placeholder --
broadcast both as identity so RViz can resolve the live point cloud
(under camera_init) and the robot model (under odom -> base_footprint,
a separate branch FAST-LIO2 knows nothing about) while mapping.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('libbot_localization'),
                'launch', 'localization_3d.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
