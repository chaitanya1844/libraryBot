#!/usr/bin/env python3
"""Drives the book-collection loop: for every book prop in
config/book_catalog.yaml, navigate to it, run a scripted arm trajectory to
"grasp" it (weld it to the gripper via libbot_gazebo_plugins' attach_link
service -- Gazebo Classic has no real suction/friction grasp), carry it to
a randomly chosen shelf from config/shelf_catalog.yaml, and release it
there (detach_link + a set_entity_state teleport to rest it at the
shelf's own pose, since the arm is scripted rather than IK-planned and
won't necessarily place it with mm precision).

    ros2 run libbot_manipulation task_manager_node.py

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

# Canned joint-space poses (radians). This arm is scripted, not IK-planned
# (see module docstring): "reach" is just a forward-and-down pose that
# clears a desk in front of the robot, not a pose computed for any
# specific book's exact location.
POSE_STOW = [0.0, -1.4, 1.6, 0.4]
POSE_REACH = [0.0, -0.15, 0.65, 0.55]
POSE_LIFT = [0.0, -0.9, 1.0, 0.3]

ROBOT_MODEL_NAME = 'libbot'
GRIPPER_LINK_NAME = 'gripper_link'
BOOK_LINK_NAME = 'link'


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('libbot_task_manager')
        self.declare_parameter('book_catalog', '')
        self.declare_parameter('shelf_catalog', '')

        pkg_share = get_package_share_directory('libbot_manipulation')
        book_path = self.get_parameter('book_catalog').value or \
            f'{pkg_share}/config/book_catalog.yaml'
        shelf_path = self.get_parameter('shelf_catalog').value or \
            f'{pkg_share}/config/shelf_catalog.yaml'

        with open(book_path) as f:
            self.books = yaml.safe_load(f)['books']
        with open(shelf_path) as f:
            self.shelves = yaml.safe_load(f)['shelves']
        random.shuffle(self.books)

        self.get_logger().info(
            f'Loaded {len(self.books)} books and {len(self.shelves)} shelves')

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
        req.link1_name = GRIPPER_LINK_NAME
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
        req.link1_name = GRIPPER_LINK_NAME
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

        self.get_logger().info('All books collected and shelved.')


def main():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.run()
    finally:
        node.navigator.lifecycleShutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
