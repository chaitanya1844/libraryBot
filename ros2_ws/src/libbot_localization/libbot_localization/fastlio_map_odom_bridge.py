#!/usr/bin/env python3
"""Makes FAST-LIO2 Nav2's localization source instead of AMCL.

Nav2 needs a `map -> odom` transform (the localization correction) on
top of a *continuous* `odom -> base_footprint` (from the EKF, wheel+IMU
fusion, unaffected by localization jumps -- this is what keeps the local
costmap from jittering every time the global pose estimate updates).
AMCL provides exactly that map->odom correction; this node computes the
same thing from FAST-LIO2 instead:

    map -> odom  =  (map -> body)  *  inverse(odom -> base_footprint)

so that composing map->odom->base_footprint always equals FAST-LIO2's
own body pose estimate.

FAST-LIO2 publishes its estimate on /Odometry as a pose in its own
"camera_init" frame -- wherever the robot happened to be when
fastlio_mapping started, *not* necessarily the 2D map's origin. If you
drove around during a mapping run and then start localization from
wherever you ended up (the common case -- there's no reason to drive
back to the exact original spawn point first), camera_init and the 2D
map's true origin are two different points, and treating them as
coincident (this bridge's original behavior) makes the 3D cloud and the
2D map disagree by exactly that offset -- FastLioCloud/FastLioPath
visibly not lining up with the Map display, and Nav2 planning against a
map that doesn't match where FAST-LIO2 thinks the robot actually is.

Fix: same tool AMCL used for the same problem. Publish a "2D Pose
Estimate" in RViz once, at the robot's actual current position on the
map, right after starting localization. This bridge listens on
/initialpose and uses it to solve for the one-time offset between
"map" and FAST-LIO2's "camera_init":

    map -> camera_init  =  (map -> true_body_now) * inverse(camera_init -> body_now)

held fixed from then on (camera_init doesn't drift once FAST-LIO2 has
initialized, so this offset doesn't need to be recomputed). Every
subsequent /Odometry sample is composed through that fixed offset
before being used to correct map->odom. Before any /initialpose is
received, the offset defaults to identity -- correct only if the robot
happens to already be sitting at the map's origin pose, same caveat the
old spawn-coincidence assumption had.

Run this instead of AMCL (nav2_amcl), alongside map_server (still needed
to serve the static map for Nav2's costmaps) and FAST-LIO2 itself -- see
localization_fastlio.launch.py.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def pose_to_matrix(position, orientation):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(
        [orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
    m[:3, 3] = [position.x, position.y, position.z]
    return m


def transform_to_matrix(t):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(
        [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).as_matrix()
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


def matrix_to_transform(m, out: TransformStamped):
    out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = m[:3, 3]
    q = Rotation.from_matrix(m[:3, :3]).as_quat()
    out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = q


class FastLioMapOdomBridge(Node):

    def __init__(self):
        super().__init__('fastlio_map_odom_bridge')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._broadcaster = TransformBroadcaster(self)

        # map -> camera_init, solved for once an /initialpose arrives (see
        # _on_initial_pose); identity until then.
        self._map_to_camera_init = np.eye(4)
        self._latest_camera_init_to_body = None

        self.create_subscription(Odometry, '/Odometry', self._on_odometry, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initial_pose, 10)
        self.get_logger().info(
            'fastlio_map_odom_bridge ready -- publishing map->%s from /Odometry. '
            'Use RViz\'s "2D Pose Estimate" once to align the 3D map with the 2D '
            'one if they were started from different poses.' % self._odom_frame)

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        if self._latest_camera_init_to_body is None:
            self.get_logger().warn(
                'Got /initialpose before any /Odometry from FAST-LIO2 -- ignoring, try again '
                'once FAST-LIO2 has started publishing.')
            return
        map_to_true_body = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        self._map_to_camera_init = map_to_true_body @ np.linalg.inv(self._latest_camera_init_to_body)
        self.get_logger().info('Re-aligned map<->camera_init from 2D Pose Estimate')

    def _on_odometry(self, msg: Odometry):
        self._latest_camera_init_to_body = pose_to_matrix(
            msg.pose.pose.position, msg.pose.pose.orientation)

        # Also broadcast map->camera_init directly (not just folded into
        # map->odom below) so RViz can place FAST-LIO2's own outputs that
        # are stamped in its "camera_init" frame -- /cloud_registered,
        # /path -- under the "map" Fixed Frame. Without this, those two
        # displays either don't render at all or render somewhere
        # unrelated to the 2D map, which is exactly "3D map not
        # overlaying the 2D map" as seen in RViz.
        camera_init_out = TransformStamped()
        camera_init_out.header.stamp = msg.header.stamp
        camera_init_out.header.frame_id = 'map'
        camera_init_out.child_frame_id = 'camera_init'
        matrix_to_transform(self._map_to_camera_init, camera_init_out)
        self._broadcaster.sendTransform(camera_init_out)

        try:
            odom_to_base = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time())
        except Exception as exc:
            self.get_logger().warn(
                'no %s->%s yet (EKF not up?): %s' % (self._odom_frame, self._base_frame, exc),
                throttle_duration_sec=2.0)
            return

        map_to_body = self._map_to_camera_init @ self._latest_camera_init_to_body
        odom_to_base_m = transform_to_matrix(odom_to_base.transform)
        map_to_odom = map_to_body @ np.linalg.inv(odom_to_base_m)

        out = TransformStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.child_frame_id = self._odom_frame
        matrix_to_transform(map_to_odom, out)
        self._broadcaster.sendTransform(out)


def main():
    rclpy.init()
    node = FastLioMapOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
