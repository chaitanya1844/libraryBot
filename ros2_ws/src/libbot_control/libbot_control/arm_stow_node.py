#!/usr/bin/env python3
"""One-shot: sends the arm to its stowed pose and exits.

Run once shortly after arm_controller activates (see control.launch.py)
so the arm never sits at its spawn/zero pose -- with a side-mounted arm,
all-joints-zero extends the whole arm straight out from the chassis
(~0.7m), which is both a poor look and made mapping runs harder to
thread through tight corners/aisles. This is the same pose
libbot_manipulation/task_manager_node.py returns the arm to between
pick/place cycles (POSE_STOW here is a plain copy, not imported, since
this runs standalone before that package is necessarily even relevant).
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_pitch_joint']
POSE_STOW = [0.0, -1.4, 1.6, 0.4]


def main():
    rclpy.init()
    node = Node('arm_stow_node')
    client = ActionClient(node, FollowJointTrajectory, '/libbot/arm_controller/follow_joint_trajectory')

    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().warn('arm_controller action server not available, giving up on stow')
        node.destroy_node()
        rclpy.shutdown()
        return

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = POSE_STOW
    point.time_from_start = Duration(sec=2)
    goal.trajectory.points = [point]

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    goal_handle = send_future.result()
    if goal_handle is not None and goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
        node.get_logger().info('Arm stowed')
    else:
        node.get_logger().warn('Stow goal was not accepted')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
