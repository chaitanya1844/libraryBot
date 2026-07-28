#!/usr/bin/env python3
"""Tracks specific named Gazebo models (book props, shelves, anything with
a model name in the world) and republishes each one's live pose as both a
TF frame (`map` -> `<item_name>`) and a small RViz marker, so their actual
position in the sim is visible/usable without hand-recording coordinates.

Reads ground truth straight from /libbot/model_states (published by the
libbot_gazebo_ros_state world plugin, see library.world) rather than the
static x/y/z recorded in libbot_manipulation's book_catalog.yaml -- that
catalog is a fixed snapshot for approach-pose planning, this stays live
so a book that's been picked up and moved (once manipulation is running)
still shows its current position, not where it spawned.

Coordinates from /libbot/model_states are Gazebo's world frame, used
directly as `map` frame here with no extra transform -- the same
assumption libbot_manipulation/task_manager_node.py already makes for
its approach_pose NavigateToPose goals (frame_id: 'map').

`item_names` is a runtime-settable parameter (not just a launch-time
default) specifically so more names can be added later without a
restart:

    ros2 param set /item_pose_publisher item_names \\
        "['BookE_01_006', 'BookE_01_007', 'BookE_01_030', 'BookE_01_017']"
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class ItemPosePublisher(Node):

    def __init__(self):
        super().__init__('item_pose_publisher')
        self.declare_parameter('item_names', [
            'BookE_01_006', 'BookE_01_007', 'BookE_01_030', 'BookE_01_017',
        ])
        self.declare_parameter('model_states_topic', '/libbot/model_states')
        self.declare_parameter('marker_topic', '/item_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('marker_scale', 0.12)
        self.declare_parameter('color_rgb', [1.0, 0.55, 0.0])

        self._frame_id = self.get_parameter('frame_id').value
        self._scale = self.get_parameter('marker_scale').value
        self._color_rgb = self.get_parameter('color_rgb').value

        self._tf_broadcaster = TransformBroadcaster(self)
        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 1)
        self.create_subscription(
            ModelStates, self.get_parameter('model_states_topic').value,
            self._on_model_states, 10)

        self._missing_warned = set()
        self.get_logger().info(
            f"item_pose_publisher ready -- tracking "
            f"{self.get_parameter('item_names').value} from "
            f"{self.get_parameter('model_states_topic').value}, "
            f"publishing TF '{self._frame_id}' -> <item_name> and "
            f"MarkerArray on {self.get_parameter('marker_topic').value}.")

    def _on_model_states(self, msg: ModelStates):
        item_names = self.get_parameter('item_names').value
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        stamp = self.get_clock().now().to_msg()

        markers = MarkerArray()
        for marker_id, item_name in enumerate(item_names):
            idx = name_to_index.get(item_name)
            if idx is None:
                if item_name not in self._missing_warned:
                    self.get_logger().warn(
                        f"'{item_name}' not found in model_states -- "
                        f"check the name against library.world.")
                    self._missing_warned.add(item_name)
                continue
            self._missing_warned.discard(item_name)
            pose = msg.pose[idx]

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._frame_id
            t.child_frame_id = item_name
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation
            self._tf_broadcaster.sendTransform(t)

            markers.markers.append(self._make_sphere(item_name, marker_id, pose, stamp))
            markers.markers.append(self._make_label(item_name, marker_id, pose, stamp))

        if markers.markers:
            self._marker_pub.publish(markers)

    def _make_sphere(self, item_name, marker_id, pose, stamp):
        m = Marker()
        m.header.frame_id = self._frame_id
        m.header.stamp = stamp
        m.ns = 'item_markers'
        m.id = marker_id * 2
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose
        m.scale.x = m.scale.y = m.scale.z = self._scale
        r, g, b = self._color_rgb
        m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
        return m

    def _make_label(self, item_name, marker_id, pose, stamp):
        m = Marker()
        m.header.frame_id = self._frame_id
        m.header.stamp = stamp
        m.ns = 'item_labels'
        m.id = marker_id * 2 + 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose = pose
        m.pose.position.z += self._scale * 1.5
        m.scale.z = self._scale * 1.2
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        m.text = item_name
        return m


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
