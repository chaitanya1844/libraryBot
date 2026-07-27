#!/usr/bin/env python3
"""Republishes /libbot/lidar_3d/points with self-hits on the robot's own
body dropped: a radial cylinder around the sensor's own vertical axis
(the sensor-mast puck the lidar sits on top of -- radially symmetric, so
every azimuth ray grazes its rim at close range regardless of robot
pose) plus a box covering the stowed arm (mounted close enough to the
lidar, on the same side of the chassis, that no stow pose puts the whole
arm outside the sensor's 360-degree ring -- pose alone can't fix that
one either).

Both are static because the geometry generating them doesn't move
during normal operation: the mast is a fixed part of the chassis, and
the arm only leaves its stowed box while libbot_manipulation's task
manager is actively picking/placing (not while driving between poses).
If the arm needs to be usable *while* the lidar is actively relied on
for something (unlikely today), this filter would need to become
joint-state-aware instead of a fixed box.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Lidar3DSelfFilter(Node):

    def __init__(self):
        super().__init__('lidar_3d_self_filter')
        self.declare_parameter('input_topic', '/libbot/lidar_3d/points')
        self.declare_parameter('output_topic', '/libbot/lidar_3d/points_filtered')
        # Box in lidar_3d_link frame covering the stowed arm (measured
        # via tf2 from lidar_3d_link to each arm link with the arm at
        # POSE_STOW, plus ~5cm margin on every side).
        self.declare_parameter('box_min', [-0.05, -0.40, -0.45])
        self.declare_parameter('box_max', [0.55, -0.05, 0.15])
        # Cylinder around the lidar's own vertical (Z) axis covering the
        # sensor-mast puck it sits on top of -- radially symmetric, so
        # every azimuth needs the same exclusion regardless of robot
        # heading. Radius measured empirically (self-hit ring showed up
        # at ~0.2-0.28m horizontal distance) with margin; z_max is
        # slightly above the lidar's own origin since the mast's top
        # edge/curve grazes rays a bit higher than the mast body itself.
        self.declare_parameter('mast_radius', 0.6)
        self.declare_parameter('mast_z_max', 0.12)

        self._box_min = np.array(self.get_parameter('box_min').value)
        self._box_max = np.array(self.get_parameter('box_max').value)
        self._mast_radius = self.get_parameter('mast_radius').value
        self._mast_z_max = self.get_parameter('mast_z_max').value

        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self._on_cloud, 10)

    def _on_cloud(self, msg: PointCloud2):
        pts = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
        if pts.shape[0] == 0:
            self._pub.publish(msg)
            return

        xyz = pts[:, :3]
        inside_arm_box = np.all((xyz >= self._box_min) & (xyz <= self._box_max), axis=1)
        radius = np.hypot(xyz[:, 0], xyz[:, 1])
        inside_mast = (radius < self._mast_radius) & (xyz[:, 2] < self._mast_z_max)
        kept = pts[~(inside_arm_box | inside_mast)]

        out = point_cloud2.create_cloud(msg.header, msg.fields, kept)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DSelfFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
