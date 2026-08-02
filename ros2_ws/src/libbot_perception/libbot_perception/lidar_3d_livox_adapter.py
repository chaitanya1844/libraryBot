#!/usr/bin/env python3
"""Republishes /libbot/lidar_3d/points (plain x/y/z/intensity from Gazebo's
ray sensor) as a livox_ros_driver2/msg/CustomMsg on /livox/lidar -- the
same message type and topic name a real Livox Mid360's driver
(livox_ros_driver2) publishes, so FAST-LIO2's `lidar_type: 1` (Livox)
ingestion path and any other Livox-aware tooling work against this sim
exactly as they would against real hardware.

Gazebo's ray sensor captures the whole cloud at one simulation instant
(there's no real per-point capture time / motion distortion to recover),
so `offset_time` and `line` here are synthesized from each point's
geometry rather than measured, mirroring the same approach the old
Velodyne-format adapter used:
  - line: vertical angle bucketed into `channels` bins across the
    sensor's known vertical FOV (must match libbot.urdf.xacro's
    lidar_3d instantiation). A real Mid360 has no fixed
    lines/channels (non-repetitive rosette scan) -- this is purely an
    artifact of the underlying Gazebo ray sensor's row/column model.
  - offset_time: zero for every point, because every point really was
    captured at the same instant. This previously synthesized a ramp
    (azimuth mapped across one frame period, the way a real spinning
    sensor reports it), which is wrong by construction: FAST-LIO2
    treats offset_time as ground truth and back-propagates each point
    along the IMU-integrated trajectory to undo motion distortion, so
    a fabricated ramp over an instantaneous capture makes it "undo"
    skew that was never there, shearing each cloud in proportion to
    yaw rate.
    Measured honestly, though, this is a correctness fix and not a
    drift fix: over two in-place revolutions at 0.5 rad/s the shear is
    only ~3 deg across azimuth, and translation drift with the ramp
    (0.185/0.041/0.071 m over three trials) versus without it
    (0.205/0.104/0.086 m) is the same within run-to-run noise. Don't
    expect this to visibly improve a map -- for that, see the
    duplicate-adapter leak fixed in libbot_bringup/scripts/switch_mode.sh,
    which is what actually wrecks one (three adapters publishing the
    same scan with different timebases measured 0.347 m).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from livox_ros_driver2.msg import CustomMsg, CustomPoint


class Lidar3DLivoxAdapter(Node):

    def __init__(self):
        super().__init__('lidar_3d_livox_adapter')
        self.declare_parameter('channels', 40)
        self.declare_parameter('vfov_deg', 59.0)
        self.declare_parameter('input_topic', '/libbot/lidar_3d/points')
        self.declare_parameter('output_topic', '/livox/lidar')

        self._channels = self.get_parameter('channels').value
        self._half_vfov = math.radians(self.get_parameter('vfov_deg').value) / 2.0

        self._pub = self.create_publisher(
            CustomMsg, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self._on_cloud, 10)

    def _on_cloud(self, msg: PointCloud2):
        points = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
        if points.shape[0] == 0:
            return

        x, y, z, intensity = points[:, 0], points[:, 1], points[:, 2], points[:, 3]

        vertical_angle = np.arctan2(z, np.sqrt(x * x + y * y))
        line = np.clip(
            ((vertical_angle + self._half_vfov) / (2 * self._half_vfov) * self._channels).astype(np.uint8),
            0, self._channels - 1)

        reflectivity = np.clip(intensity, 0, 255).astype(np.uint8)

        out = CustomMsg()
        out.header = msg.header
        # The capture stamp, not "now" -- with every offset_time zero, timebase
        # is the one time FAST-LIO2 has for this cloud, so it has to be when
        # the scan was taken rather than when this callback happened to run.
        out.timebase = (msg.header.stamp.sec * 10**9) + msg.header.stamp.nanosec
        out.point_num = x.shape[0]
        out.lidar_id = 0
        out.points = [
            CustomPoint(
                offset_time=0,
                x=float(x[i]), y=float(y[i]), z=float(z[i]),
                reflectivity=int(reflectivity[i]),
                tag=0,
                line=int(line[i]),
            )
            for i in range(x.shape[0])
        ]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DLivoxAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
