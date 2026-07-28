#!/usr/bin/env python3
"""Live 2D occupancy grid preview while FAST-LIO2 mapping is running.

FAST-LIO2 replaced slam_toolbox as the mapping source (see
mapping.launch.py), which means there's no live 2D map being built the
way slam_toolbox used to show one -- only a 3D point cloud
(/Laser_map, the *accumulated* map; not /cloud_registered, which is
just the current scan). The real 2D map only gets generated once, after
mapping, via pcd_to_2d_map.py against the saved PCD.

This node re-projects the same accumulating /Laser_map cloud into a 2D
occupancy grid every `update_period_sec`, purely so RViz's Map display
has something to show *while* mapping -- a live preview, not a
replacement for pcd_to_2d_map.py's final output (which also gets to use
the full, unthrottled point cloud rather than whatever's arrived by
each preview tick).

/Laser_map is published in FAST-LIO2's own "camera_init" frame, NOT
"map" -- fastlio_map_odom_bridge.py solves a real (usually non-identity)
map -> camera_init offset (translation *and* rotation), so this must
actually transform each point into "map" frame before gridding it, not
just relabel the raw camera_init-frame coordinates as if they were
already map-frame (which only ever looked right by coincidence, back
when that offset happened to be identity).
"""
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class MappingPreviewNode(Node):

    def __init__(self):
        super().__init__('mapping_preview_node')
        self.declare_parameter('input_topic', '/Laser_map')
        self.declare_parameter('output_topic', '/mapping_preview_map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('z_min', 0.05)
        self.declare_parameter('z_max', 0.9)
        self.declare_parameter('occupied_min_points', 2)
        self.declare_parameter('update_period_sec', 2.0)

        self._resolution = self.get_parameter('resolution').value
        self._z_min = self.get_parameter('z_min').value
        self._z_max = self.get_parameter('z_max').value
        self._occupied_min_points = self.get_parameter('occupied_min_points').value
        self._frame_id = self.get_parameter('frame_id').value

        self._latest_cloud = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('output_topic').value, 1)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self._on_cloud, 5)
        self.create_timer(self.get_parameter('update_period_sec').value, self._update)

        self.get_logger().info(
            "mapping_preview_node ready -- live 2D preview of %s on %s, "
            'not a substitute for pcd_to_2d_map.py once mapping is done.'
            % (self.get_parameter('input_topic').value,
               self.get_parameter('output_topic').value))

    def _on_cloud(self, msg: PointCloud2):
        self._latest_cloud = msg

    def _update(self):
        if self._latest_cloud is None:
            return

        pts = point_cloud2.read_points_numpy(
            self._latest_cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return

        try:
            t = self._tf_buffer.lookup_transform(
                self._frame_id, self._latest_cloud.header.frame_id, Time(),
                timeout=Duration(seconds=0.2))
        except Exception as exc:
            self.get_logger().warn(
                'no %s -> %s yet: %s' % (self._frame_id, self._latest_cloud.header.frame_id, exc),
                throttle_duration_sec=2.0)
            return
        tr = t.transform.translation
        rot = t.transform.rotation
        R = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
        pts = pts @ R.T + np.array([tr.x, tr.y, tr.z])

        band = pts[(pts[:, 2] >= self._z_min) & (pts[:, 2] <= self._z_max)]
        if band.shape[0] == 0:
            return

        x_min, y_min = pts[:, 0].min(), pts[:, 1].min()
        x_max, y_max = pts[:, 0].max(), pts[:, 1].max()
        res = self._resolution
        width = max(1, int(np.ceil((x_max - x_min) / res)) + 1)
        height = max(1, int(np.ceil((y_max - y_min) / res)) + 1)

        cols = np.clip(((band[:, 0] - x_min) / res).astype(np.int64), 0, width - 1)
        rows = np.clip(((band[:, 1] - y_min) / res).astype(np.int64), 0, height - 1)

        counts = np.zeros((height, width), dtype=np.int32)
        np.add.at(counts, (rows, cols), 1)

        grid = np.full((height, width), -1, dtype=np.int8)
        seen = counts > 0
        grid[seen] = 0
        grid[counts >= self._occupied_min_points] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(x_min)
        msg.info.origin.position.y = float(y_min)
        msg.info.origin.orientation.w = 1.0
        # OccupancyGrid data is row-major starting from the origin (no
        # vertical flip needed here, unlike pcd_to_2d_map.py's pgm output
        # -- that flip is a pgm-file convention, not an OccupancyGrid one).
        msg.data = grid.flatten().tolist()
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = MappingPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
