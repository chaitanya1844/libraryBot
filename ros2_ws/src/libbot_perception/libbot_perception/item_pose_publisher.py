#!/usr/bin/env python3
"""Tracks specific named Gazebo models (book props, shelves, anything with
a model name in the world) and republishes each one's live pose as both a
TF frame (`map` -> `<item_name>`) and a small RViz marker, so their actual
position in the sim is visible/usable without hand-recording coordinates,
and lines up exactly with whatever Nav2 itself is currently navigating
against (needed so a task manager can send NavigateToPose goals straight
at these coordinates).

Reads ground truth straight from /libbot/model_states (published by the
libbot_gazebo_ros_state world plugin, see library.world) rather than the
static x/y/z recorded in libbot_manipulation's book_catalog.yaml -- that
catalog is a fixed snapshot for approach-pose planning, this stays live
so a book that's been picked up and moved (once manipulation is running)
still shows its current position, not where it spawned.

Two earlier versions of this node got the map-frame placement wrong:
1. Broadcasting raw model_states coordinates directly as children of
   `map`, assuming map's origin coincides with Gazebo's world origin --
   true right when mapping starts, not once localization's ICP
   correction has shifted map relative to odom.
2. Anchoring a `world` frame to `odom` via a *hardcoded* spawn-pose
   offset (simulation.launch.py's x/y/yaw launch args). That assumes
   odom's origin never moves from the spawn pose -- false the moment
   anything teleports the robot without also resetting the odometry
   plugin's internal integrator (e.g. the control panel's "Reset
   Position" button), after which odom silently drifts out of sync
   with that assumption and every item ends up offset by however much.

Fixed by computing map -> item fresh every cycle from two *live* facts,
no fixed assumptions at all:
  - this robot's own ground-truth pose in Gazebo's world frame (from
    the same /libbot/model_states message, filtering for
    robot_model_name)
  - a live TF lookup of map -> base_footprint (the exact same
    transform Nav2's BT/controller resolve through map -> odom ->
    base_footprint)

    map_to_item = map_to_base @ inverse(world_to_base) @ world_to_item

`item_names`/`shelf_names` are runtime-settable parameters (not just
launch-time defaults) specifically so more names can be added later
without a restart:

    ros2 param set /item_pose_publisher item_names \\
        "['BookE_01_006', 'BookE_01_007', 'BookE_01_030', 'BookE_01_017']"
"""
import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def pose_to_matrix(pose: Pose) -> np.ndarray:
    m = np.eye(4)
    o = pose.orientation
    m[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
    m[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return m


def transform_to_matrix(t) -> np.ndarray:
    m = np.eye(4)
    r = t.rotation
    m[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


def matrix_to_pose(m: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = m[:3, 3]
    q = Rotation.from_matrix(m[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


class ItemPosePublisher(Node):

    def __init__(self):
        super().__init__('item_pose_publisher')
        self.declare_parameter('item_names', [
            'BookE_01_006', 'BookE_01_007', 'BookE_01_030', 'BookE_01_017',
        ])
        # The big center shelves, tracked the same way but with a bigger
        # cube marker instead of a small sphere so they read as
        # landmarks rather than pickable items.
        self.declare_parameter('shelf_names', [
            'BookshelfA_01_001', 'BookshelfA_01_002',
            'BookshelfA_01_003', 'BookshelfA_01_004',
        ])
        self.declare_parameter('model_states_topic', '/libbot/model_states')
        self.declare_parameter('marker_topic', '/item_markers')
        self.declare_parameter('robot_model_name', 'libbot')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('marker_scale', 0.12)
        self.declare_parameter('color_rgb', [1.0, 0.55, 0.0])
        # x/y swapped from the naive (long-axis-along-x) guess: the
        # AWS retail bookshelf model's own long axis is local Y, not X,
        # so using (1.0, 0.4, ...) rendered the cube marker rotated 90
        # degrees off from the real shelf's footprint.
        self.declare_parameter('shelf_marker_scale', [0.4, 1.0, 1.8])
        # map<->world is conceptually a fixed calibration constant, but
        # recomputing it fresh every cycle from two independently-timed
        # live sources (Gazebo's instantaneous ground truth vs. a TF
        # lookup that lags slightly behind during active motion, since
        # it flows through EKF fusion + rate-limited ICP corrections)
        # let that transient mismatch leak straight into it while the
        # robot drives -- items visibly jittering/shifting in sync with
        # the robot's own motion, even though their true world position
        # never changed. Exponential smoothing damps that out the same
        # way it already fixed ICP correction jitter in
        # fastlio_map_odom_bridge.py.
        self.declare_parameter('calibration_smoothing_alpha', 0.15)
        self.declare_parameter('shelf_color_rgb', [0.15, 0.45, 0.95])

        self._robot_model_name = self.get_parameter('robot_model_name').value
        self._base_frame = self.get_parameter('base_frame').value
        self._scale = self.get_parameter('marker_scale').value
        self._color_rgb = self.get_parameter('color_rgb').value
        self._shelf_scale = self.get_parameter('shelf_marker_scale').value
        self._shelf_color_rgb = self.get_parameter('shelf_color_rgb').value
        self._calib_alpha = self.get_parameter('calibration_smoothing_alpha').value
        self._smoothed_map_to_world_xy = None
        self._smoothed_map_to_world_z = None
        self._smoothed_map_to_world_yaw = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 1)
        self.create_subscription(
            ModelStates, self.get_parameter('model_states_topic').value,
            self._on_model_states, 10)

        self._missing_warned = set()
        self._map_unavailable_warned = False
        self.get_logger().info(
            f"item_pose_publisher ready -- tracking items "
            f"{self.get_parameter('item_names').value} and shelves "
            f"{self.get_parameter('shelf_names').value} from "
            f"{self.get_parameter('model_states_topic').value}, "
            f"publishing TF 'map' -> <name> (recomputed live every "
            f"cycle from Gazebo ground truth + map->{self._base_frame}, "
            f"no fixed spawn-pose assumption) and MarkerArray on "
            f"{self.get_parameter('marker_topic').value}.")

    def _smooth_map_to_world(self, raw: np.ndarray) -> np.ndarray:
        xy = raw[:2, 3]
        z = raw[2, 3]
        yaw = Rotation.from_matrix(raw[:3, :3]).as_euler('zyx')[0]

        if self._smoothed_map_to_world_xy is None:
            self._smoothed_map_to_world_xy = xy
            self._smoothed_map_to_world_z = z
            self._smoothed_map_to_world_yaw = yaw
        else:
            a = self._calib_alpha
            self._smoothed_map_to_world_xy = self._smoothed_map_to_world_xy + a * (xy - self._smoothed_map_to_world_xy)
            self._smoothed_map_to_world_z = self._smoothed_map_to_world_z + a * (z - self._smoothed_map_to_world_z)
            dyaw = (yaw - self._smoothed_map_to_world_yaw + np.pi) % (2 * np.pi) - np.pi
            self._smoothed_map_to_world_yaw = self._smoothed_map_to_world_yaw + a * dyaw

        smoothed = np.eye(4)
        smoothed[:3, :3] = Rotation.from_euler('z', self._smoothed_map_to_world_yaw).as_matrix()
        smoothed[:2, 3] = self._smoothed_map_to_world_xy
        smoothed[2, 3] = self._smoothed_map_to_world_z
        return smoothed

    def _on_model_states(self, msg: ModelStates):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        robot_idx = name_to_index.get(self._robot_model_name)
        if robot_idx is None:
            self.get_logger().warn(
                f"robot model '{self._robot_model_name}' not in model_states",
                throttle_duration_sec=5.0)
            return

        try:
            map_to_base_tf = self._tf_buffer.lookup_transform(
                'map', self._base_frame, Time())
        except Exception as exc:
            if not self._map_unavailable_warned:
                self.get_logger().warn(
                    f"no map->{self._base_frame} yet (start mapping/localization "
                    f"first) -- item markers withheld until it's available: {exc}")
                self._map_unavailable_warned = True
            return
        self._map_unavailable_warned = False

        map_to_base = transform_to_matrix(map_to_base_tf.transform)
        world_to_base = pose_to_matrix(msg.pose[robot_idx])
        # map_to_world: composes Gazebo's raw world-frame coordinates
        # straight into whatever "map" currently is, live, with no
        # fixed assumptions -- self-corrects after any teleport/reset
        # since it's rebuilt from ground truth every single cycle.
        raw_map_to_world = map_to_base @ np.linalg.inv(world_to_base)
        map_to_world = self._smooth_map_to_world(raw_map_to_world)

        item_names = self.get_parameter('item_names').value
        shelf_names = self.get_parameter('shelf_names').value
        stamp = self.get_clock().now().to_msg()

        markers = MarkerArray()
        # id ranges kept disjoint (items: 0..999, shelves: 1000..1999) so
        # the two categories never collide even as more names are added.
        for marker_id, name in enumerate(item_names):
            markers.markers.extend(self._track_one(
                name, marker_id, name_to_index, msg, map_to_world, stamp,
                'item_markers', 'item_labels', Marker.SPHERE,
                (self._scale,) * 3, self._color_rgb, self._scale, 0.0))
        for marker_id, name in enumerate(shelf_names):
            # Shelf models' own origin is near their base (close to
            # ground), not their geometric center -- centering a tall
            # cube marker there put half of it visibly floating above
            # where the shelf actually is. Shift up by half the cube's
            # height so it rests on the origin instead of straddling it.
            markers.markers.extend(self._track_one(
                name, 1000 + marker_id, name_to_index, msg, map_to_world, stamp,
                'shelf_markers', 'shelf_labels', Marker.CUBE,
                tuple(self._shelf_scale), self._shelf_color_rgb,
                max(self._shelf_scale), self._shelf_scale[2] / 2.0))

        if markers.markers:
            self._marker_pub.publish(markers)

    def _track_one(self, name, marker_id, name_to_index, msg, map_to_world, stamp,
                    marker_ns, label_ns, marker_type, scale_xyz, color_rgb, label_offset,
                    marker_z_offset):
        idx = name_to_index.get(name)
        if idx is None:
            if name not in self._missing_warned:
                self.get_logger().warn(
                    f"'{name}' not found in model_states -- "
                    f"check the name against library.world.")
                self._missing_warned.add(name)
            return []
        self._missing_warned.discard(name)

        map_to_item = map_to_world @ pose_to_matrix(msg.pose[idx])
        pose = matrix_to_pose(map_to_item)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = name
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self._tf_broadcaster.sendTransform(t)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = marker_ns
        marker.id = marker_id * 2
        marker.type = marker_type
        marker.action = Marker.ADD
        # Deep-copied, not aliased to `pose` -- marker/label each need
        # their own independent Pose object since label.pose gets its
        # z shifted below; a shared reference here previously let that
        # shift silently corrupt the marker's own z too (a real, large
        # bug for shelves: label_offset=1.8 meant a spurious +2.7m).
        marker.pose = copy.deepcopy(pose)
        marker.pose.position.z += marker_z_offset
        marker.scale.x, marker.scale.y, marker.scale.z = scale_xyz
        r, g, b = color_rgb
        marker.color = ColorRGBA(r=r, g=g, b=b, a=0.85)

        label = Marker()
        label.header.frame_id = 'map'
        label.header.stamp = stamp
        label.ns = label_ns
        label.id = marker_id * 2 + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = copy.deepcopy(pose)
        label.pose.position.z += marker_z_offset + label_offset * 1.5
        label.scale.z = self._scale * 1.2
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        label.text = name

        return [marker, label]


def main():
    rclpy.init()
    node = ItemPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
