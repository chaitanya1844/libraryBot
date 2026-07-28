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

RViz's Fixed Frame is "map", but nothing publishes it at all during
mapping unless we run something here too -- without it, RViz can't
resolve any transform and renders nothing. This run is what *defines*
the map, so map == camera_init's origin is correct (fastlio_map_odom_bridge
with no pcd_map_path leaves that offset at identity, same as
localization before its first correction). But map -> odom must NOT
also be identity: odom's origin is the robot's *spawn* pose, while
camera_init's origin is wherever the robot was when fastlio_mapping
itself started -- if you'd driven at all before clicking "Start
Mapping", those two are already offset, and blindly identity-transforming
both under "map" (an earlier version of this file did exactly that)
makes RobotModel (on the odom branch) and FastLioCloud/FastLioPath (on
the camera_init branch) render in two different, unaligned places.
fastlio_map_odom_bridge already computes map -> odom correctly from the
*actual* live camera_init -> body and odom -> base transforms (see its
own docstring) -- reusing it here instead of a second blind identity
transform is what actually fixes that.

Also runs mapping_preview_node.py: FAST-LIO2 has no live 2D map the way
slam_toolbox used to show one (the real 2D map only exists once,
generated after the fact by pcd_to_2d_map.py against the saved PCD) --
this republishes the same accumulating cloud as a live OccupancyGrid
preview every couple seconds, purely for visual feedback that mapping is
actually accumulating data while you drive.

/Laser_map is remapped to /Laser_map_mapping here (SetRemap, applies
through the nested include below to fastlio_mapping's own publisher):
localization_fastlio.launch.py's fastlio_mapping instance publishes
/Laser_map too (its own fresh live SLAM session, no "localize-only" mode
exists), and RViz's FastLioCloud display needs to tell that apart from
this mapping run's -- see libbot_nav.rviz (green while mapping, yellow
while localizing). fast_lio_localization's global_localization.py
doesn't consume /Laser_map at all (only /cloud_registered, /Odometry),
so remapping it doesn't affect anything functional, only which RViz
display picks it up.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='libbot_localization',
            executable='fastlio_map_odom_bridge.py',
            name='fastlio_map_odom_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='libbot_mapping',
            executable='mapping_preview_node.py',
            name='mapping_preview_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'input_topic': '/Laser_map_mapping'}],
        ),

        SetRemap(src='/Laser_map', dst='/Laser_map_mapping'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('libbot_localization'),
                'launch', 'localization_3d.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
