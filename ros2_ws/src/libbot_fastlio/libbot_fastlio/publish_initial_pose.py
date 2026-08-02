#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import tf_transformations


class PublishInitialPose(Node):
    def __init__(self):
        super().__init__("publish_initial_pose")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def publish_pose(self, x, y, z, roll, pitch, yaw):
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        initial_pose = PoseWithCovarianceStamped()
        # Keyword args, not positional: rosidl-generated message classes
        # accept keywords only, so Pose(Point(*xyz), Quaternion(*quat))
        # raises TypeError before anything is ever published.
        initial_pose.pose.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"

        # Wait for global_localization to actually be subscribed before
        # publishing. This process exits immediately afterwards, so a
        # message sent before discovery completes is simply dropped and
        # the seed silently never arrives.
        deadline = time.time() + 10.0
        while (self.pub_pose.get_subscription_count() == 0
               and time.time() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pub_pose.get_subscription_count() == 0:
            self.get_logger().warn(
                "No subscriber on /initialpose after 10s -- publishing anyway, "
                "but nothing is listening for the seed.")

        self.pub_pose.publish(initial_pose)
        # And give the middleware a moment to actually put it on the wire.
        end = time.time() + 1.0
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Initial Pose: {x} {y} {z} {yaw} {pitch} {roll}")


def main(args=None):
    rclpy.init(args=args)
    node = PublishInitialPose()

    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("yaw", type=float)
    parser.add_argument("pitch", type=float)
    parser.add_argument("roll", type=float)
    args = parser.parse_args()

    node.publish_pose(args.x, args.y, args.z, args.roll, args.pitch, args.yaw)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
