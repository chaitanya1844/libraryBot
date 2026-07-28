#!/usr/bin/env python3
"""Sorts a specific, named subset of books into the center bookshelves,
randomly assigning each one a target shelf -- same pick/place mechanics
as task_manager_node.py (weld-via-attach_link "grasp", scripted arm
trajectory, no real IK), but scoped to book_names/shelf_names instead of
every book_catalog.yaml entry, and attaches each book to book_tray_link
(the robot's front intake tray, i.e. its "basket") instead of directly
to gripper_link, so it visually rides along on the tray between pickup
and drop-off rather than staying stuck to the gripper the whole time.

    ros2 run libbot_manipulation book_sorter_node.py

book_names/shelf_names are runtime-settable parameters -- more books can
be added later the same way item_pose_publisher's item_names is:

    ros2 param set /libbot_book_sorter book_names \\
        "['BookE_01_006', 'BookE_01_007', 'BookE_01_030', 'BookE_01_017']"

Requires Nav2 (autonomy.launch.py) already active and localized -- this
node is a NavigateToPose client, not a replacement for it.

book_catalog.yaml/shelf_catalog.yaml's x/y/yaw are recorded in Gazebo's
raw world frame, not `map` -- those only coincide when map's origin
happens to line up with world's (true right after a fresh mapping
session, not reliably true once localization's ICP has corrected map
relative to odom, exactly the bug fixed in item_pose_publisher.py's
docstring). Sending them to Nav2 as-is under frame_id 'map' sent the
robot to wherever that *stale* assumption put them, not where the book/
shelf actually is on the currently active map -- which is why the
planned path looked wrong. Fixed the same way: convert each catalog
pose into the *current* map frame live, right before every nav goal,
using this robot's own live ground truth (model_states) plus a live TF
lookup of map -> base_footprint.
"""
import math
import random
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial.transform import Rotation

import yaml
from ament_index_python.packages import get_package_share_directory

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from libbot_gazebo_plugins.srv import AttachLink, DetachLink


def _pose_to_matrix(position, orientation) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(
        [orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
    m[:3, 3] = [position.x, position.y, position.z]
    return m


def _transform_to_matrix(t) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(
        [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).as_matrix()
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


def _xyzyaw_to_matrix(x, y, yaw) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    m[:2, 3] = [x, y]
    return m


ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_pitch_joint']

# Same canned joint-space poses as task_manager_node.py -- this arm is
# scripted, not IK-planned, see that module's docstring.
POSE_STOW = [0.0, -1.4, 1.6, 0.4]
POSE_REACH = [0.0, -0.15, 0.65, 0.55]
POSE_LIFT = [0.0, -0.9, 1.0, 0.3]

ROBOT_MODEL_NAME = 'libbot'
BASKET_LINK_NAME = 'book_tray_link'
BOOK_LINK_NAME = 'link'


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class BookSorterNode(Node):

    def __init__(self):
        super().__init__('libbot_book_sorter')
        self.declare_parameter('book_catalog', '')
        self.declare_parameter('shelf_catalog', '')
        self.declare_parameter('book_names', [
            'BookE_01_006', 'BookE_01_007', 'BookE_01_030', 'BookE_01_017',
        ])
        self.declare_parameter('shelf_names', [
            'BookshelfA_01_001', 'BookshelfA_01_002',
            'BookshelfA_01_003', 'BookshelfA_01_004',
        ])
        self.declare_parameter('robot_model_name', 'libbot')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('model_states_topic', '/libbot/model_states')

        pkg_share = get_package_share_directory('libbot_manipulation')
        book_path = self.get_parameter('book_catalog').value or \
            f'{pkg_share}/config/book_catalog.yaml'
        shelf_path = self.get_parameter('shelf_catalog').value or \
            f'{pkg_share}/config/shelf_catalog.yaml'

        with open(book_path) as f:
            all_books = {b['name']: b for b in yaml.safe_load(f)['books']}
        with open(shelf_path) as f:
            all_shelves = {s['name']: s for s in yaml.safe_load(f)['shelves']}

        book_names = self.get_parameter('book_names').value
        shelf_names = self.get_parameter('shelf_names').value
        self.books = []
        for name in book_names:
            if name not in all_books:
                self.get_logger().warn(f"'{name}' not in book_catalog.yaml -- skipping")
                continue
            self.books.append(all_books[name])
        self.shelves = []
        for name in shelf_names:
            if name not in all_shelves:
                self.get_logger().warn(f"'{name}' not in shelf_catalog.yaml -- skipping")
                continue
            self.shelves.append(all_shelves[name])

        if not self.shelves:
            raise RuntimeError('No valid target shelves -- check shelf_names against shelf_catalog.yaml')

        random.shuffle(self.books)
        self.get_logger().info(
            f'Sorting {len(self.books)} books across {len(self.shelves)} shelves: '
            f'{[b["name"] for b in self.books]} -> {[s["name"] for s in self.shelves]}')

        self._robot_model_name = self.get_parameter('robot_model_name').value
        self._base_frame = self.get_parameter('base_frame').value
        self._latest_model_states = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            ModelStates, self.get_parameter('model_states_topic').value,
            self._on_model_states, 10)

        cb_group = ReentrantCallbackGroup()
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/libbot/arm_controller/follow_joint_trajectory',
            callback_group=cb_group)
        self.attach_client = self.create_client(
            AttachLink, '/libbot/attach_link', callback_group=cb_group)
        self.detach_client = self.create_client(
            DetachLink, '/libbot/detach_link', callback_group=cb_group)
        self.set_state_client = self.create_client(
            SetEntityState, '/libbot/set_entity_state', callback_group=cb_group)

        self.navigator = BasicNavigator()

    # ---------------------------------------------------------------- arm
    def _send_arm_trajectory(self, waypoints, seconds_per_point=1.5):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        t = 0.0
        for wp in waypoints:
            t += seconds_per_point
            point = JointTrajectoryPoint()
            point.positions = list(wp)
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t % 1.0) * 1e9)
            goal.trajectory.points.append(point)

        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('arm_controller action server not available')
            return False

        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm trajectory rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    # ------------------------------------------------------------- attach
    def _attach_book(self, book_name):
        req = AttachLink.Request()
        req.model1_name = ROBOT_MODEL_NAME
        req.link1_name = BASKET_LINK_NAME
        req.model2_name = book_name
        req.link2_name = BOOK_LINK_NAME
        self.attach_client.wait_for_service(timeout_sec=10.0)
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return bool(result and result.success)

    def _detach_book(self, book_name):
        req = DetachLink.Request()
        req.model1_name = ROBOT_MODEL_NAME
        req.link1_name = BASKET_LINK_NAME
        req.model2_name = book_name
        req.link2_name = BOOK_LINK_NAME
        self.detach_client.wait_for_service(timeout_sec=10.0)
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return bool(result and result.success)

    def _rest_book_on_shelf(self, book_name, shelf):
        req = SetEntityState.Request()
        req.state.name = book_name
        req.state.pose.position.x = shelf['pose']['x']
        req.state.pose.position.y = shelf['pose']['y']
        # A bit above the shelf's own base pose so it settles onto the
        # shelf surface under gravity instead of spawning inside it.
        req.state.pose.position.z = shelf['pose']['z'] + 0.9
        req.state.reference_frame = 'world'
        self.set_state_client.wait_for_service(timeout_sec=10.0)
        future = self.set_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    # ------------------------------------------------------------ ground truth
    def _on_model_states(self, msg: ModelStates):
        self._latest_model_states = msg

    def _wait_for_ground_truth(self, timeout_sec=15.0):
        deadline = time.time() + timeout_sec
        while self._latest_model_states is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._latest_model_states is None:
            self.get_logger().warn(
                f'no {self.get_parameter("model_states_topic").value} yet after '
                f'{timeout_sec}s -- nav goals may be sent uncorrected.')

    def _map_to_world(self):
        """Live map<->world calibration, recomputed fresh (not cached) so
        it can't go stale like the earlier hardcoded-spawn-pose bug did
        (see module docstring): map_to_base (from a live TF lookup, same
        source Nav2 itself resolves through) composed with this robot's
        own live Gazebo ground-truth pose."""
        if self._latest_model_states is None:
            return None
        names = self._latest_model_states.name
        if self._robot_model_name not in names:
            return None
        idx = names.index(self._robot_model_name)
        world_to_base = _pose_to_matrix(
            self._latest_model_states.pose[idx].position,
            self._latest_model_states.pose[idx].orientation)
        try:
            map_to_base_tf = self._tf_buffer.lookup_transform(
                'map', self._base_frame, Time())
        except Exception as exc:
            self.get_logger().warn(f'no map->{self._base_frame} yet: {exc}', throttle_duration_sec=2.0)
            return None
        map_to_base = _transform_to_matrix(map_to_base_tf.transform)
        return map_to_base @ np.linalg.inv(world_to_base)

    def _world_pose_to_map(self, x, y, yaw):
        """Converts a book_catalog.yaml/shelf_catalog.yaml pose (raw
        Gazebo world-frame coordinates) into the current map frame, so
        Nav2 goals land on where the book/shelf actually is on whatever
        map is currently active -- not wherever a stale map==world
        assumption would put them."""
        map_to_world = self._map_to_world()
        if map_to_world is None:
            self.get_logger().warn('map<->world calibration unavailable, using raw coordinates')
            return x, y, yaw
        map_pose = map_to_world @ _xyzyaw_to_matrix(x, y, yaw)
        mx, my = map_pose[:2, 3]
        myaw = Rotation.from_matrix(map_pose[:3, :3]).as_euler('zyx')[0]
        return mx, my, myaw

    # ------------------------------------------------------------- navigation
    def _navigate_to_catalog_pose(self, x, y, yaw):
        mx, my, myaw = self._world_pose_to_map(x, y, yaw)
        return self._navigate_to(mx, my, myaw)

    def _navigate_to(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.navigator.goToPose(pose)
        while not self.navigator.isTaskComplete():
            time.sleep(0.2)
        return self.navigator.getResult() == TaskResult.SUCCEEDED

    # ------------------------------------------------------------------ run
    def run(self):
        self._wait_for_ground_truth()
        for book in self.books:
            name = book['name']
            ap = book['approach_pose']
            self.get_logger().info(f'Collecting {name}')

            if not self._navigate_to_catalog_pose(ap['x'], ap['y'], ap['yaw']):
                self.get_logger().warn(f'Nav to {name} failed, skipping')
                continue

            self._send_arm_trajectory([POSE_REACH])
            if not self._attach_book(name):
                self.get_logger().warn(f'Attach failed for {name}, skipping')
                self._send_arm_trajectory([POSE_STOW])
                continue
            self._send_arm_trajectory([POSE_LIFT, POSE_STOW])

            shelf = random.choice(self.shelves)
            sap = shelf['approach_pose']
            self.get_logger().info(f'Delivering {name} to {shelf["name"]}')

            if not self._navigate_to_catalog_pose(sap['x'], sap['y'], sap['yaw']):
                self.get_logger().warn(f'Nav to {shelf["name"]} failed, releasing in place')

            self._send_arm_trajectory([POSE_REACH])
            self._detach_book(name)
            self._rest_book_on_shelf(name, shelf)
            self._send_arm_trajectory([POSE_STOW])

        self.get_logger().info('All requested books sorted.')


def main():
    rclpy.init()
    node = BookSorterNode()
    try:
        node.run()
    finally:
        node.navigator.lifecycleShutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
