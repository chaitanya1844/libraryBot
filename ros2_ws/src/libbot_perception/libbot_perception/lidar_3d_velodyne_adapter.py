#!/usr/bin/env python3
"""Republishes /libbot/lidar_3d/points (plain x/y/z/intensity from Gazebo's
ray sensor) as a cloud carrying the extra `ring` and `time` per-point
fields FAST-LIO's Velodyne point type requires
(velodyne_ros::Point: x,y,z,intensity,time,ring -- see
~/fastlio_ws/src/FAST_LIO/src/preprocess.h).

Gazebo's ray sensor captures the whole cloud at one simulation instant
(there's no real per-point capture time / motion distortion to recover),
so `ring` and `time` here are synthesized from each point's geometry
rather than measured:
  - ring: vertical angle bucketed into `channels` bins across the
    sensor's known vertical FOV (must match libbot.urdf.xacro's
    lidar_3d instantiation: 16 channels, +-15deg).
  - time: horizontal angle mapped across one frame period (1/hz), the
    same convention a real spinning lidar's driver uses, so FAST-LIO's
    per-ring time-ordering logic sees monotonically increasing values.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class Lidar3DVelodyneAdapter(Node):

    def __init__(self):
        super().__init__('lidar_3d_velodyne_adapter')
        self.declare_parameter('channels', 16)
        self.declare_parameter('vfov_deg', 30.0)
        self.declare_parameter('hz', 5.0)
        self.declare_parameter('input_topic', '/libbot/lidar_3d/points')
        self.declare_parameter('output_topic', '/libbot/lidar_3d/points_velodyne')

        self._channels = self.get_parameter('channels').value
        self._half_vfov = math.radians(self.get_parameter('vfov_deg').value) / 2.0
        self._frame_period = 1.0 / self.get_parameter('hz').value

        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self._on_cloud, 10)

    def _on_cloud(self, msg: PointCloud2):
        points = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
        if points.shape[0] == 0:
            return

        x, y, z, intensity = points[:, 0], points[:, 1], points[:, 2], points[:, 3]

        vertical_angle = np.arctan2(z, np.sqrt(x * x + y * y))
        ring = np.clip(
            ((vertical_angle + self._half_vfov) / (2 * self._half_vfov) * self._channels).astype(np.uint16),
            0, self._channels - 1)

        horizontal_angle = np.arctan2(y, x) + math.pi  # 0..2pi
        time = (horizontal_angle / (2 * math.pi) * self._frame_period).astype(np.float32)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
        ]
        cloud_arr = np.zeros(x.shape[0], dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32), ('time', np.float32), ('ring', np.uint16),
            ('_pad', np.uint16),
        ])
        cloud_arr['x'] = x
        cloud_arr['y'] = y
        cloud_arr['z'] = z
        cloud_arr['intensity'] = intensity
        cloud_arr['time'] = time
        cloud_arr['ring'] = ring

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = cloud_arr.shape[0]
        out.fields = fields
        out.is_bigendian = False
        out.point_step = cloud_arr.itemsize
        out.row_step = out.point_step * out.width
        out.is_dense = True
        out.data = cloud_arr.tobytes()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DVelodyneAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
