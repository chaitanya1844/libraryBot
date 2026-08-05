#!/usr/bin/env python3
"""Paints FAST-LIO2's registered lidar map with color from the D435i RGB
camera, for a presentation-friendly colored point cloud -- much cheaper
than standing up a full camera+lidar SLAM system (R3LIVE) alongside the
FAST-LIO2 pipeline that's already tuned and working here: this just
projects each already-registered lidar point into the camera image and
samples its color, accumulating into a running colored map.

Per /cloud_registered scan (already in camera_init/map-anchored frame):
  1. Look up camera_init -> camera optical frame at the scan's own
     timestamp (both TF chains -- FAST-LIO2's camera_init->body and
     robot_state_publisher's static body->camera -- are already
     published; nothing new needed there).
  2. Transform the scan into the camera frame and project with the
     camera's own intrinsics (pinhole, camera_info's K).
  3. Any point landing inside the image bounds and in front of the
     camera gets that pixel's RGB; points outside the ~69deg FOV (most
     of a 360deg lidar scan, at any lidar-only edge/behind-the-robot
     range) are dropped rather than left uncolored -- half-colored
     patchwork reads worse for a demo than a smaller solidly-colored
     cloud that grows as the robot looks around.

Accumulated into a running Open3D cloud, voxel-downsampled periodically
to bound memory the same way global_localization.py already does for
the localization map, and published live for RViz. A Trigger service
(mirroring FAST-LIO2's own /map_save) dumps it to a PCD on demand.
"""
import os

import numpy as np
import open3d as o3d
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


def transform_to_matrix(t):
    from scipy.spatial.transform import Rotation
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(
        [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).as_matrix()
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


class LidarCameraColorizer(Node):

    def __init__(self):
        super().__init__('lidar_camera_colorizer')
        self.declare_parameter('cloud_topic', '/cloud_registered')
        self.declare_parameter('image_topic', '/libbot/camera/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/libbot/camera/camera/camera_info')
        self.declare_parameter('output_topic', '/libbot/colored_map/points')
        self.declare_parameter('map_frame', 'camera_init')
        self.declare_parameter('voxel_size', 0.03)
        # Accumulated cloud is re-voxelized every this-many scans rather
        # than every single one -- voxel_down_sample over a
        # multi-hundred-thousand-point cloud every scan (up to ~10Hz
        # from FAST-LIO2) would burn CPU nothing else here needs; the
        # cloud only grows a bounded amount between re-voxelizations.
        self.declare_parameter('revoxelize_every_n', 10)
        self.declare_parameter('default_save_path', os.path.expanduser(
            '~/libraryBot/ros2_ws/src/libbot_fastlio/pcd_maps/latest_colored.pcd'))

        self._cloud_topic = self.get_parameter('cloud_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._voxel_size = self.get_parameter('voxel_size').value
        self._revoxelize_every_n = self.get_parameter('revoxelize_every_n').value
        self._default_save_path = self.get_parameter('default_save_path').value

        self._bridge = CvBridge()
        self._latest_image = None
        self._K = None
        self._image_frame = None

        self._tf_buffer = Buffer()
        # spin_thread=True: lookup_transform() below blocks briefly
        # (timeout=) waiting for a not-yet-arrived TF sample, which would
        # deadlock a single-threaded node against its own /tf
        # subscription callback otherwise -- this runs that subscription
        # on its own thread instead.
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._accum_xyz = np.empty((0, 3), dtype=np.float64)
        self._accum_rgb = np.empty((0, 3), dtype=np.float64)
        self._scans_since_voxelize = 0

        # Camera publishes at ~15Hz best-effort like most Gazebo camera
        # plugins -- match reliability or this subscription never gets
        # the sensor QoS's samples.
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self._on_camera_info, 10)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, sensor_qos)
        self.create_subscription(
            PointCloud2, self._cloud_topic, self._on_cloud, 10)

        self._pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, 1)
        self.create_service(Trigger, 'colored_map_save', self._on_save)

        self.create_timer(1.0, self._publish)

        self.get_logger().info(
            f'lidar_camera_colorizer ready -- painting {self._cloud_topic} from '
            f"{self.get_parameter('image_topic').value}")

    def _on_camera_info(self, msg: CameraInfo):
        self._K = np.array(msg.k).reshape(3, 3)
        self._image_frame = msg.header.frame_id

    def _on_image(self, msg: Image):
        self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def _on_cloud(self, msg: PointCloud2):
        if self._latest_image is None or self._K is None:
            return

        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return

        try:
            tf = self._tf_buffer.lookup_transform(
                self._image_frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=0.2))
        except Exception as exc:
            self.get_logger().warn(
                f'no {msg.header.frame_id}->{self._image_frame} TF at scan time yet: {exc}',
                throttle_duration_sec=2.0)
            return
        cloud_to_cam = transform_to_matrix(tf.transform)

        pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])
        cam_pts = (cloud_to_cam @ pts_h.T).T[:, :3]

        in_front = cam_pts[:, 2] > 0.05
        cam_pts = cam_pts[in_front]
        pts_world = pts[in_front]
        if cam_pts.shape[0] == 0:
            return

        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        u = (fx * cam_pts[:, 0] / cam_pts[:, 2] + cx).astype(np.int32)
        v = (fy * cam_pts[:, 1] / cam_pts[:, 2] + cy).astype(np.int32)

        h, w = self._latest_image.shape[:2]
        in_bounds = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        if not in_bounds.any():
            return

        u, v = u[in_bounds], v[in_bounds]
        colors = self._latest_image[v, u].astype(np.float64) / 255.0
        world_pts = pts_world[in_bounds].astype(np.float64)

        self._accum_xyz = np.vstack([self._accum_xyz, world_pts])
        self._accum_rgb = np.vstack([self._accum_rgb, colors])
        self._scans_since_voxelize += 1

        if self._scans_since_voxelize >= self._revoxelize_every_n:
            self._voxelize()
            self._scans_since_voxelize = 0

    def _voxelize(self):
        if self._accum_xyz.shape[0] == 0:
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self._accum_xyz)
        pcd.colors = o3d.utility.Vector3dVector(self._accum_rgb)
        pcd = pcd.voxel_down_sample(self._voxel_size)
        self._accum_xyz = np.asarray(pcd.points)
        self._accum_rgb = np.asarray(pcd.colors)

    def _publish(self):
        if self._accum_xyz.shape[0] == 0:
            return
        rgb_u32 = (
            (np.clip(self._accum_rgb[:, 0] * 255, 0, 255).astype(np.uint32) << 16)
            | (np.clip(self._accum_rgb[:, 1] * 255, 0, 255).astype(np.uint32) << 8)
            | np.clip(self._accum_rgb[:, 2] * 255, 0, 255).astype(np.uint32))
        rgb_f32 = rgb_u32.view(np.float32)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._map_frame
        cloud_arr = np.column_stack([self._accum_xyz, rgb_f32]).astype(np.float32)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg = point_cloud2.create_cloud(header, fields, cloud_arr)
        self._pub.publish(msg)

    def _on_save(self, request, response):
        self._voxelize()
        if self._accum_xyz.shape[0] == 0:
            response.success = False
            response.message = 'No colored points accumulated yet'
            return response
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self._accum_xyz)
        pcd.colors = o3d.utility.Vector3dVector(self._accum_rgb)
        path = self._default_save_path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        o3d.io.write_point_cloud(path, pcd)
        response.success = True
        response.message = f'Saved {len(pcd.points)} colored points to {path}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraColorizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
