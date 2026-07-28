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

1. **Plain mapping** (no prior map, `use_external_localization:=false`,
   the default): auto-aligned once at startup from the current
   odom -> base_footprint pose (see _on_odometry), since this run *is*
   what defines the map -- "wherever the robot is right now" is as good
   a map origin as any. Also accepts RViz's "2D Pose Estimate" at any
   time afterward as a manual re-alignment.
2. **Localizing against a saved map** (`use_external_localization:=true`,
   what localization_fastlio.launch.py sets): this run's camera_init is
   a brand new, arbitrary origin with no relationship to whatever old
   session's frame the saved map/PCD were actually recorded in --
   "wherever the robot is now" is *not* a valid guess at that
   relationship. A single local ICP pass from that blind guess isn't
   enough to reliably converge to the *correct* alignment rather than a
   wrong one either (tried that here first; it wasn't reliable). Instead,
   this subscribes to /map_to_odom from fast_lio_localization's
   global_localization.py (a real port of HViktorTsoi/FAST_LIO_LOCALIZATION,
   see that package's own docstring) -- it does proper coarse-to-fine ICP
   against the saved PCD, seeded by RViz's "2D Pose Estimate" the same way
   AMCL needs one, and keeps re-registering continuously afterward. Its
   `/map_to_odom` pose *is* map -> camera_init directly: its "odom" is
   FAST-LIO2's own /Odometry pose (this project's camera_init -> body),
   not this project's EKF-based odom frame at all -- don't let the name
   collide with this file's own `odom_frame` param, they're unrelated.

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
        # FAST-LIO2's "body" frame is physically wherever the IMU is
        # mounted (extrinsic_T/R in fast_lio_libbot.yaml is lidar-relative-
        # to-IMU, not lidar-relative-to-base) -- base_frame -> this is a
        # large, fixed offset (0.76m in z on this robot, measured), not
        # something to treat as zero.
        self.declare_parameter('body_frame', 'imu_link')
        self.declare_parameter('use_external_localization', False)
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._body_frame = self.get_parameter('body_frame').value
        self._use_external_localization = self.get_parameter('use_external_localization').value
        # base_frame -> body_frame is fixed (both come from
        # robot_state_publisher's static URDF joints), so this is looked
        # up once and cached rather than every callback.
        self._base_to_body = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._broadcaster = TransformBroadcaster(self)

        # map -> camera_init: identity is WRONG as a starting value in
        # plain mapping mode, not just a placeholder -- camera_init's
        # origin is wherever FAST-LIO2's body (IMU) frame physically was
        # when fastlio_mapping started, well above the ground (~0.7m+ on
        # this robot). _on_odometry's auto-align corrects it from the
        # first real odom -> base_footprint + camera_init -> body pair.
        # In use_external_localization mode this is overwritten entirely
        # by /map_to_odom instead (see module docstring).
        self._map_to_camera_init = np.eye(4)
        self._auto_aligned = False
        self._latest_camera_init_to_body = None

        self.create_subscription(Odometry, '/Odometry', self._on_odometry, 20)

        if self._use_external_localization:
            self.create_subscription(
                Odometry, '/map_to_odom', self._on_external_map_to_odom, 10)
            self.get_logger().info(
                'fastlio_map_odom_bridge ready -- waiting for /map_to_odom from '
                'fast_lio_localization (needs RViz\'s "2D Pose Estimate" to seed it).')
        else:
            self.create_subscription(
                PoseWithCovarianceStamped, '/initialpose', self._on_initial_pose, 10)
            self.get_logger().info(
                'fastlio_map_odom_bridge ready -- publishing map->%s from /Odometry, '
                'auto-aligned to the current pose (mapping mode, no prior map).'
                % self._odom_frame)

    def _on_external_map_to_odom(self, msg: Odometry):
        # fast_lio_localization's "odom" is FAST-LIO2's own /Odometry pose
        # -- i.e. this project's camera_init -> body -- not this file's
        # own odom_frame param. This message would directly *be*
        # map -> camera_init, except: its point-to-point ICP
        # (registration_at_scale) is unconstrained in z/roll/pitch, same
        # risk already found and fixed in this bridge's own (now retired)
        # ICP -- measured drifting z by over a meter on this ground robot.
        # Trust it for x/y/yaw (that's the actual localization signal);
        # for z/roll/pitch, use the same geometry-based reference the
        # plain-mapping auto-align already computes, since a ground robot
        # has no reason for that to ever change.
        if self._latest_camera_init_to_body is None or self._base_to_body is None:
            return
        try:
            odom_to_base = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time())
        except Exception:
            return
        odom_to_base_m = transform_to_matrix(odom_to_base.transform)

        received = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        yaw = Rotation.from_matrix(received[:3, :3]).as_euler('zyx')[0]
        reference = odom_to_base_m @ self._base_to_body @ np.linalg.inv(self._latest_camera_init_to_body)

        constrained = np.eye(4)
        constrained[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
        constrained[:2, 3] = received[:2, 3]
        constrained[2, 3] = reference[2, 3]

        self._map_to_camera_init = constrained
        self._auto_aligned = True

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        if self._use_external_localization:
            return  # fast_lio_localization's global_localization.py handles this instead.
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
            # Same placeholder either way: "wherever the robot currently
            # is" as the map origin. Correct by construction in plain
            # mapping mode (this run defines the map); in
            # use_external_localization mode it's just a placeholder so
            # RobotModel/TF have *something* to resolve immediately --
            # publishing nothing at all until the first /map_to_odom
            # arrived (an earlier version of this) meant RobotModel
            # didn't render for however long it took to click "2D Pose
            # Estimate" and get an accepted ICP fitness, which could be a
            # while. _on_external_map_to_odom overwrites this with the
            # real correction the moment one arrives; until then this is
            # wrong exactly the same way AMCL's map->odom is wrong before
            # its own first initial pose, not broken.
            self._map_to_camera_init = (
                odom_to_base_m @ self._base_to_body @ np.linalg.inv(self._latest_camera_init_to_body))
            self._auto_aligned = True
            self.get_logger().info(
                'Auto-aligned map<->camera_init from current odom->%s pose (z-corrected)%s.'
                % (self._base_frame,
                   ' -- placeholder until ICP corrects it' if self._use_external_localization else ''))

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
