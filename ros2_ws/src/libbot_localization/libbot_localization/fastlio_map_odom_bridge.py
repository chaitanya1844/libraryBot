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
"camera_init" frame (wherever it happened to initialize -- in this sim,
the robot's spawn pose). This bridge never actually uses "camera_init"
as a TF frame name; it broadcasts directly as "map", i.e. it treats
FAST-LIO2's init pose as coincident with the 2D map's origin-frame
convention. That's a simplifying assumption that holds as long as the
robot spawns at the pose the map/AMCL setup already assumed (see
simulation.launch.py's spawn x/y/yaw and amcl.yaml's initial_pose,
which already have to agree with each other for the same reason).

Run this instead of AMCL (nav2_amcl), alongside map_server (still needed
to serve the static map for Nav2's costmaps) and FAST-LIO2 itself -- see
localization_fastlio.launch.py.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
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

        self.create_subscription(Odometry, '/Odometry', self._on_odometry, 20)
        self.get_logger().info(
            'fastlio_map_odom_bridge ready -- publishing map->%s from /Odometry' % self._odom_frame)

    def _on_odometry(self, msg: Odometry):
        try:
            odom_to_base = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time())
        except Exception as exc:
            self.get_logger().warn(
                'no %s->%s yet (EKF not up?): %s' % (self._odom_frame, self._base_frame, exc),
                throttle_duration_sec=2.0)
            return

        map_to_body = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
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
