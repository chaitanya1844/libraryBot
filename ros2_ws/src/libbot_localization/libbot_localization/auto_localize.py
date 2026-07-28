#!/usr/bin/env python3
"""One-shot global relocalization: finds the robot's pose in a saved PCD
map without needing a precise manual "2D Pose Estimate" click first.

fast_lio_localization's global_localization.py only ever does *local*
ICP (registration_at_scale, point-to-point, starting from whatever pose
you seed it with) -- it has no way to recover if that seed is
meaningfully wrong, and manual clicks kept converging to the same wrong
spot here regardless of where they were actually placed, suggesting the
basin of convergence from a bad seed is narrow. This does real global
registration instead (FPFH features + RANSAC, no seed needed at all):
computes local geometric features for the current body-frame scan and
the saved map, finds candidate correspondences between them, and
RANSAC-fits a transform -- standard Open3D global registration, the
same category of algorithm real SLAM relocalization stacks use for
exactly this "where am I with no prior" problem.

Once a good-fitness global fit is found, publishes it on /initialpose
(std PoseWithCovarianceStamped) -- global_localization.py picks that up
exactly like a manual RViz click and takes over with its own continuous
local ICP refinement from there. This node then exits; it's meant to
run once at the start of a localization session, not continuously (its
own registration is much heavier per-attempt than the local ICP it's
seeding).

Usage: after Start Localization (map_server + fast_lio_localization +
fastlio_map_odom_bridge already running, per localization_fastlio.launch.py):

    ros2 run libbot_localization auto_localize.py --ros-args \\
        -p pcd_map_path:=/path/to/name.pcd
"""
import sys

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class AutoLocalizeNode(Node):

    def __init__(self):
        super().__init__('auto_localize')
        self.declare_parameter('pcd_map_path', '')
        self.declare_parameter('voxel_size', 0.15)
        self.declare_parameter('fitness_threshold', 0.3)

        pcd_map_path = self.get_parameter('pcd_map_path').value
        if not pcd_map_path:
            self.get_logger().error('pcd_map_path is required')
            sys.exit(1)

        voxel = self.get_parameter('voxel_size').value
        self.get_logger().info(f"Loading '{pcd_map_path}'...")
        self._target = o3d.io.read_point_cloud(pcd_map_path)
        self._target = self._target.voxel_down_sample(voxel)
        self._target_fpfh = self._compute_fpfh(self._target, voxel)
        self.get_logger().info(f'Target map: {len(self._target.points)} points after downsampling.')

        self._pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._accumulated = []
        self._done = False
        self.create_subscription(PointCloud2, '/cloud_registered_body', self._on_scan, 5)
        # A single scan (~1000 points, robot stationary while seeding) is
        # too sparse for FPFH feature matching to find anything --
        # measured fitness=0.000, complete failure. Body frame is
        # IMU-centered, not accumulating world-frame drift, so merging a
        # few messages' worth while the robot sits still is safe and
        # gives FPFH much more geometry to work with. Message-count
        # trigger, not a wall/sim-time timer: under use_sim_time, a
        # timer's very first large clock jump (this node's own clock
        # starting at 0, then jumping to the sim's current, already-large
        # time on the first /clock message) satisfies any short period
        # instantly -- measured firing in 2ms instead of the intended 3s.
        self._target_scan_count = 10
        self.get_logger().info(
            f'Accumulating {self._target_scan_count} body-frame scans (/cloud_registered_body)...')

    def _compute_fpfh(self, pcd, voxel_size):
        radius_normal = voxel_size * 2
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        radius_feature = voxel_size * 5
        return o3d.pipelines.registration.compute_fpfh_feature(
            pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    def _on_scan(self, msg: PointCloud2):
        if self._done:
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] > 0:
            self._accumulated.append(pts)
        if len(self._accumulated) >= self._target_scan_count:
            self._run_registration()

    def _run_registration(self):
        if self._done:
            return
        self._done = True
        pts = np.concatenate(self._accumulated, axis=0)
        self.get_logger().info(f'Accumulated {pts.shape[0]} points -- running global registration...')

        voxel = self.get_parameter('voxel_size').value
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        source = source.voxel_down_sample(voxel)
        source_fpfh = self._compute_fpfh(source, voxel)

        distance_threshold = voxel * 1.5
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source, self._target, source_fpfh, self._target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            4,
            [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
            ],
            o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 1000))

        self.get_logger().info(
            f'Global registration fitness={result.fitness:.3f}, rmse={result.inlier_rmse:.3f}m')
        threshold = self.get_parameter('fitness_threshold').value
        if result.fitness < threshold:
            self.get_logger().error(
                f'Fitness below {threshold} -- not publishing. Try driving to a more '
                'distinctive part of the map (near walls/corners, not open floor) and '
                'run this again, or fall back to a manual "2D Pose Estimate".')
            rclpy.shutdown()
            return

        # source was already in body frame; result.transformation is
        # body -> map directly (registration_ransac's "source" param).
        xyz = result.transformation[:3, 3]
        quat = Rotation.from_matrix(result.transformation[:3, :3]).as_quat()

        msg_out = PoseWithCovarianceStamped()
        msg_out.header.frame_id = 'map'
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.pose.pose.position.x = float(xyz[0])
        msg_out.pose.pose.position.y = float(xyz[1])
        msg_out.pose.pose.position.z = 0.0
        msg_out.pose.pose.orientation.x = float(quat[0])
        msg_out.pose.pose.orientation.y = float(quat[1])
        msg_out.pose.pose.orientation.z = float(quat[2])
        msg_out.pose.pose.orientation.w = float(quat[3])
        # A few publishes, not one -- /initialpose is Volatile QoS and
        # global_localization.py's subscription might not have been up
        # for the very first message depending on timing.
        for _ in range(5):
            self._pub.publish(msg_out)
            rclpy.spin_once(self, timeout_sec=0.3)
        self.get_logger().info(
            f'Published /initialpose from global registration: '
            f'x={xyz[0]:.2f} y={xyz[1]:.2f}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = AutoLocalizeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
