#!/usr/bin/env python3
"""Publishes a breadcrumb trail of the robot's actual traveled path, as a
nav_msgs/Path, for RViz to draw alongside Nav2's GlobalPlan/LocalPlan
(the future trajectory). Nav2 doesn't publish anything like this itself --
it only knows where it's going, not where it's been."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class TrailPublisher(Node):

    def __init__(self):
        super().__init__('trail_publisher')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('trail_topic', '/libbot/trail')
        self.declare_parameter('max_points', 3000)
        # Only append a new point once the robot has moved this far from
        # the last recorded one -- keeps the trail message small even
        # over a long mapping run, without visibly changing its shape.
        self.declare_parameter('min_point_spacing', 0.05)

        self._max_points = self.get_parameter('max_points').value
        self._min_spacing = self.get_parameter('min_point_spacing').value
        self._path = Path()
        self._last_xy = None

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(
            Path, self.get_parameter('trail_topic').value, qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 20)

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_xy is not None:
            dx, dy = x - self._last_xy[0], y - self._last_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self._min_spacing:
                return
        self._last_xy = (x, y)

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._path.poses.append(ps)
        if len(self._path.poses) > self._max_points:
            self._path.poses = self._path.poses[-self._max_points:]

        self._path.header = msg.header
        self._pub.publish(self._path)


def main():
    rclpy.init()
    node = TrailPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
