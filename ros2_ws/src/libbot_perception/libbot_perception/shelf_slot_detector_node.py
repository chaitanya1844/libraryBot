#!/usr/bin/env python3
"""Depth-camera shelf-slot inference.

Looks at a horizontal band across the middle of the RealSense D435i depth
image (the shelf level directly in front of the robot), bins it into
vertical strips, and treats the strip with the greatest median depth as
the most likely "open" spot on the shelf: a genuinely empty gap reads
farther away than a shelf packed with books, because the camera sees past
it to the shelf's back panel. This is a real, working depth heuristic --
not a placeholder -- but it is deliberately simple (no book/object
classifier), matching a shelf with no ground-truth "empty slot" markers
in the simulation. Swap in a learned detector here for higher fidelity.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from libbot_msgs.msg import ShelfSlot


class ShelfSlotDetectorNode(Node):

    def __init__(self):
        super().__init__('shelf_slot_detector_node')

        self.declare_parameter('num_bins', 12)
        self.declare_parameter('band_fraction', 0.2)
        self.declare_parameter('min_valid_depth', 0.25)
        self.declare_parameter('max_valid_depth', 3.0)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('current_shelf_id', '')
        self.declare_parameter('current_shelf_level', 0)

        self.num_bins = int(self.get_parameter('num_bins').value)
        self.band_fraction = float(self.get_parameter('band_fraction').value)
        self.min_valid_depth = float(self.get_parameter('min_valid_depth').value)
        self.max_valid_depth = float(self.get_parameter('max_valid_depth').value)
        self.target_frame = self.get_parameter('target_frame').value

        self.bridge = CvBridge()
        self.camera_info = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/libbot/camera/camera/depth/camera_info',
                                  self._on_camera_info, 10)
        self.create_subscription(Image, '/libbot/camera/camera/depth/image_raw',
                                  self._on_depth_image, qos_profile_sensor_data)

        self.pub = self.create_publisher(ShelfSlot, '/libbot/perception/shelf_slot', 10)

        self.get_logger().info('Shelf-slot detector ready (waiting for depth frames)')

    def _on_camera_info(self, msg: CameraInfo):
        if self.camera_info is None:
            self.get_logger().info('First camera_info received')
        self.camera_info = msg

    def _on_depth_image(self, msg: Image):
        if self.camera_info is None:
            self.get_logger().info('Depth frame arrived before camera_info; skipping', throttle_duration_sec=2.0)
            return

        try:
            self._process_depth(msg)
        except Exception as exc:  # noqa: BLE001 -- log and keep the node alive
            self.get_logger().error(f'Depth processing failed: {exc}')

    def _process_depth(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        h, w = depth.shape

        band_h = max(2, int(h * self.band_fraction))
        y0 = h // 2 - band_h // 2
        band = depth[y0:y0 + band_h, :]

        bin_w = w // self.num_bins
        if bin_w == 0:
            return

        bin_depths = []
        for i in range(self.num_bins):
            col_slice = band[:, i * bin_w:(i + 1) * bin_w]
            valid = col_slice[np.isfinite(col_slice) &
                               (col_slice > self.min_valid_depth) &
                               (col_slice < self.max_valid_depth)]
            bin_depths.append(float(np.median(valid)) if valid.size > 0 else float('nan'))

        valid_bins = [(i, d) for i, d in enumerate(bin_depths) if not math.isnan(d)]
        if not valid_bins:
            return

        best_i, best_depth = max(valid_bins, key=lambda t: t[1])
        depths_only = [d for _, d in valid_bins]
        mean_depth = sum(depths_only) / len(depths_only)
        spread = max(depths_only) - min(depths_only)
        confidence = 0.0 if spread < 1e-3 else min(1.0, (best_depth - mean_depth) / spread)
        confidence = max(0.0, confidence)

        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]

        px = (best_i + 0.5) * bin_w
        py = y0 + band_h / 2.0
        x = (px - cx) * best_depth / fx
        y = (py - cy) * best_depth / fy
        z = best_depth

        slot_width = (bin_w * best_depth) / fx

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        out_pose = pose
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, rclpy.time.Time())
            out_pose = PoseStamped()
            out_pose.header.frame_id = self.target_frame
            out_pose.header.stamp = msg.header.stamp
            out_pose.pose = do_transform_pose(pose.pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

        slot = ShelfSlot()
        slot.pose = out_pose
        slot.width = slot_width
        slot.confidence = confidence
        slot.shelf_id = self.get_parameter('current_shelf_id').value
        slot.shelf_level = int(self.get_parameter('current_shelf_level').value)
        self.pub.publish(slot)


def main():
    rclpy.init()
    node = ShelfSlotDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
