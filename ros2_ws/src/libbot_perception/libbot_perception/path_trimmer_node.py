#!/usr/bin/env python3
"""Republishes a nav_msgs/Path with only its last N poses -- RViz's Path
display has no "Decay Time" the way PointCloud2 does, so FAST-LIO2's own
/path (its full trajectory since mapping/localization started) only ever
grows, eventually drawing as a long red tangle across the whole map on a
long session. Trims it here instead of touching FAST-LIO2 itself (a
compiled binary in a separate overlay workspace).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class PathTrimmerNode(Node):

    def __init__(self):
        super().__init__('path_trimmer_node')
        self.declare_parameter('input_topic', '/path')
        self.declare_parameter('output_topic', '/path_trimmed')
        self.declare_parameter('max_poses', 300)

        self._max_poses = self.get_parameter('max_poses').value
        self._pub = self.create_publisher(Path, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Path, self.get_parameter('input_topic').value, self._on_path, 10)

    def _on_path(self, msg: Path):
        if len(msg.poses) > self._max_poses:
            msg.poses = msg.poses[-self._max_poses:]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathTrimmerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
