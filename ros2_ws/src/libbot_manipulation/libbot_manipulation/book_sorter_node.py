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
"""
import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import yaml
from ament_index_python.packages import get_package_share_directory

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetEntityState

from libbot_gazebo_plugins.srv import AttachLink, DetachLink


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

    # ------------------------------------------------------------- navigation
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
        for book in self.books:
            name = book['name']
            ap = book['approach_pose']
            self.get_logger().info(f'Collecting {name}')

            if not self._navigate_to(ap['x'], ap['y'], ap['yaw']):
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

            if not self._navigate_to(sap['x'], sap['y'], sap['yaw']):
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
