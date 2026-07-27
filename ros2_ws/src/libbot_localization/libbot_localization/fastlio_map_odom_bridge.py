#!/usr/bin/env python3
"""Makes FAST-LIO2 Nav2's localization source instead of AMCL.

Nav2 needs a `map -> odom` transform (the localization correction) on
top of a *continuous* `odom -> base_footprint` (from the EKF, wheel+IMU
fusion, unaffected by localization jumps -- this is what keeps the local
costmap from jittering every time the global pose estimate updates).
AMCL provides exactly that map->odom correction; this node computes the
same thing from FAST-LIO2 instead:

    map -> odom  =  (map -> body) * inverse(base -> body) * inverse(odom -> base)

so that composing map->odom->base always equals FAST-LIO2's own body
pose estimate, translated back to base_footprint via the *fixed*
base -> body offset (base_frame/body_frame params, "body" being
wherever FAST-LIO2's IMU physically is -- 0.76m up in z on this robot,
not the same point as base_footprint at all). Skipping that correction
-- treating body and base_footprint as interchangeable -- was an actual
bug here for a while: it bakes in a permanent, constant z bias between
the accumulated map and everything else, easy to mistake for ordinary
FAST-LIO2 drift since, unlike drift, it never grows or shrinks.

FAST-LIO2 publishes its estimate on /Odometry as a pose in its own
"camera_init" frame -- wherever the robot happened to be when
fastlio_mapping started, *not* necessarily the 2D map's origin. Two ways
this node solves for the actual offset between "map" and "camera_init":

0. **Automatic, at startup**: the first time both /Odometry and
   odom -> base_footprint are available, solves the same equation using
   the robot's *current* odom-frame pose as the "true" one, so
   map -> odom starts at identity right then. This matters even for
   plain mapping (no map/PCD involved yet): camera_init's origin is
   wherever FAST-LIO2's body/IMU frame physically was at startup, well
   above the ground (measured ~0.7m on this robot) -- leaving the offset
   at identity, as an earlier version of this node did, put the entire
   accumulated point cloud floating at the wrong height relative to the
   ground-referenced robot model.
1. **ICP against a saved PCD map** (set the `pcd_map_path` parameter --
   see pcd_to_2d_map.py for turning that same PCD into the 2D map Nav2's
   costmaps use). Every `icp_period_sec`, the latest body-frame scan
   (/cloud_registered_body) is transformed into the current best "map"
   guess and registered against the saved map with Open3D point-to-point
   ICP; a good-fitness result corrects `map -> camera_init` right then.
   This is the real fix for *localizing against a previous map* --
   continuous, self-correcting, matches how FAST-LIO-LOCALIZATION
   (github hku-mars ecosystem, ROS1/Python2 only, not usable here
   directly) fuses a low-rate global correction with FAST-LIO's own
   high-rate odometry.
2. **One-shot from RViz's "2D Pose Estimate"** (always available, no PCD
   needed): publishes once on /initialpose, solves
   `map -> camera_init = (map -> true_body_now) * inverse(camera_init -> body_now)`
   and holds it fixed. Same equation as the automatic startup correction,
   just re-triggerable any time (e.g. as a manual correction if ICP ever
   drifts/loses track).

Run this instead of AMCL (nav2_amcl), alongside map_server (still needed
to serve the static map for Nav2's costmaps) and FAST-LIO2 itself -- see
localization_fastlio.launch.py.
"""
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
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
        # FAST-LIO2's "body" frame is physically wherever the IMU is
        # mounted (extrinsic_T/R in fast_lio_libbot.yaml is lidar-relative-
        # to-IMU, not lidar-relative-to-base) -- base_frame -> this is a
        # large, fixed offset (0.76m in z on this robot, measured), not
        # something to treat as zero.
        self.declare_parameter('body_frame', 'imu_link')
        self.declare_parameter('pcd_map_path', '')
        self.declare_parameter('icp_period_sec', 2.0)
        self.declare_parameter('icp_max_correspondence_distance', 1.0)
        self.declare_parameter('icp_min_fitness', 0.3)
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._body_frame = self.get_parameter('body_frame').value
        # base_frame -> body_frame is fixed (both come from
        # robot_state_publisher's static URDF joints), so this is looked
        # up once and cached rather than every callback.
        self._base_to_body = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._broadcaster = TransformBroadcaster(self)

        # map -> camera_init: identity is WRONG as a starting value, not
        # just a placeholder -- camera_init's origin is wherever FAST-LIO2's
        # body (IMU) frame physically was when fastlio_mapping started,
        # which sits well above the ground (the IMU's mount height, e.g.
        # ~0.7m+ on this robot), not at floor level like odom/map. Leaving
        # this at identity was measured putting the whole accumulated map
        # cloud floating ~0.7m off from the ground-referenced robot model.
        # _auto_align_once (below) corrects it automatically from the
        # first real odom -> base_footprint + camera_init -> body pair
        # instead of waiting for a manual "2D Pose Estimate" click or ICP
        # (both of which still work afterward to refine further).
        self._map_to_camera_init = np.eye(4)
        self._auto_aligned = False
        self._latest_camera_init_to_body = None
        self._latest_body_scan = None

        self.create_subscription(Odometry, '/Odometry', self._on_odometry, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initial_pose, 10)

        pcd_map_path = self.get_parameter('pcd_map_path').value
        self._target_map = None
        if pcd_map_path:
            self._target_map = o3d.io.read_point_cloud(pcd_map_path)
            self._target_map = self._target_map.voxel_down_sample(0.1)
            if len(self._target_map.points) == 0:
                self.get_logger().error(
                    f"'{pcd_map_path}' loaded but has 0 points -- ICP correction disabled. "
                    "Was it saved via `ros2 service call /map_save std_srvs/srv/Trigger`, "
                    'with `publish.map_en: true` set in fast_lio_libbot.yaml? SIGINT/Ctrl-C '
                    "doesn't write this path at all in this ROS2 port.")
                self._target_map = None
            else:
                self.get_logger().info(
                    f"Loaded ICP target map '{pcd_map_path}' "
                    f'({len(self._target_map.points)} points after downsampling)')
                self.create_subscription(
                    PointCloud2, '/cloud_registered_body', self._on_body_scan, 5)
                self.create_timer(self.get_parameter('icp_period_sec').value, self._run_icp)

        self.get_logger().info(
            'fastlio_map_odom_bridge ready -- publishing map->%s from /Odometry. '
            'Use RViz\'s "2D Pose Estimate" once to align the 3D map with the 2D '
            'one if they were started from different poses%s.'
            % (self._odom_frame, ', or let ICP correct it automatically' if self._target_map else ''))

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        if self._latest_camera_init_to_body is None or self._base_to_body is None:
            self.get_logger().warn(
                'Got /initialpose before FAST-LIO2/TF were ready -- ignoring, try again shortly.')
            return
        # RViz's "2D Pose Estimate" sets base_footprint's pose, not body's
        # (the click is on the 2D map, which the robot footprint occupies)
        # -- go through base_to_body same as everywhere else in this file.
        map_to_true_base = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        map_to_true_body = map_to_true_base @ self._base_to_body
        self._map_to_camera_init = map_to_true_body @ np.linalg.inv(self._latest_camera_init_to_body)
        self.get_logger().info('Re-aligned map<->camera_init from 2D Pose Estimate')

    def _on_body_scan(self, msg: PointCloud2):
        self._latest_body_scan = msg

    def _run_icp(self):
        if self._latest_body_scan is None or self._latest_camera_init_to_body is None:
            return

        pts = point_cloud2.read_points_numpy(
            self._latest_body_scan, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] < 30:
            return

        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

        # Transform the body-frame scan into the current best guess at
        # "map" frame; ICP then only has to correct the *residual* error
        # in that guess, not register from scratch each time.
        guess = self._map_to_camera_init @ self._latest_camera_init_to_body
        source.transform(guess)

        result = o3d.pipelines.registration.registration_icp(
            source, self._target_map,
            self.get_parameter('icp_max_correspondence_distance').value,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint())

        if result.fitness < self.get_parameter('icp_min_fitness').value:
            self.get_logger().warn(
                f'ICP fitness {result.fitness:.2f} too low, skipping this correction',
                throttle_duration_sec=5.0)
            return

        corrected_guess = result.transformation @ guess
        self._map_to_camera_init = corrected_guess @ np.linalg.inv(self._latest_camera_init_to_body)
        self.get_logger().info(
            f'ICP correction applied (fitness={result.fitness:.2f}, rmse={result.inlier_rmse:.3f}m)',
            throttle_duration_sec=5.0)

    def _on_odometry(self, msg: Odometry):
        self._latest_camera_init_to_body = pose_to_matrix(
            msg.pose.pose.position, msg.pose.pose.orientation)

        try:
            odom_to_base = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time())
        except Exception as exc:
            self.get_logger().warn(
                'no %s->%s yet (EKF not up?): %s' % (self._odom_frame, self._base_frame, exc),
                throttle_duration_sec=2.0)
            return
        odom_to_base_m = transform_to_matrix(odom_to_base.transform)

        if self._base_to_body is None:
            try:
                base_to_body = self._tf_buffer.lookup_transform(
                    self._base_frame, self._body_frame, Time())
                self._base_to_body = transform_to_matrix(base_to_body.transform)
            except Exception as exc:
                self.get_logger().warn(
                    'no %s->%s yet (robot_state_publisher not up?): %s'
                    % (self._base_frame, self._body_frame, exc), throttle_duration_sec=2.0)
                return

        if not self._auto_aligned:
            # First real pair of (camera_init -> body, odom -> base) seen --
            # solve map -> camera_init so that map -> odom starts at
            # identity right now (this run defines the map, so "wherever
            # the robot currently is" is as good a map origin as any),
            # instead of leaving camera_init's arbitrary IMU-height-relative
            # origin baked into every point in the accumulated cloud.
            # Composes through base_to_body (NOT treating body/IMU and
            # base_footprint as the same point, which an earlier version of
            # this fix did -- that left a permanent ~0.76m z bias baked into
            # every map point, mistaken at the time for ordinary FAST-LIO2
            # drift since it never went away).
            self._map_to_camera_init = (
                odom_to_base_m @ self._base_to_body @ np.linalg.inv(self._latest_camera_init_to_body))
            self._auto_aligned = True
            self.get_logger().info(
                'Auto-aligned map<->camera_init from current odom->%s pose '
                '(z-corrected -- use "2D Pose Estimate" or ICP to refine further).'
                % self._base_frame)

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

        map_to_body = self._map_to_camera_init @ self._latest_camera_init_to_body
        map_to_base = map_to_body @ np.linalg.inv(self._base_to_body)
        map_to_odom = map_to_base @ np.linalg.inv(odom_to_base_m)

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
