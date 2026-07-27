#!/usr/bin/env python3
"""Publishes a breadcrumb trail of the robot's actual traveled path, as a
visualization_msgs/MarkerArray line strip, for RViz to draw alongside
Nav2's GlobalPlan/LocalPlan (the future trajectory). Nav2 doesn't publish
anything like this itself -- it only knows where it's going, not where
it's been.

A single LINE_STRIP marker with a per-point `colors` array (RViz/Rviz's
marker renderer honors per-vertex alpha) fades each point's opacity by
age, and points older than `max_age_sec` are dropped outright -- so the
trail reads as "recently driven", not a permanent scribble across the
whole map that never clears."""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class TrailPublisher(Node):

    def __init__(self):
        super().__init__('trail_publisher')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('trail_topic', '/libbot/trail')
        self.declare_parameter('max_points', 3000)
        self.declare_parameter('max_age_sec', 20.0)
        # Base color is deliberately dim/dark rather than a bright
        # saturated blue -- at full alpha it should already read as
        # subdued against the map, then fade to fully transparent by
        # max_age_sec.
        self.declare_parameter('color_rgb', [0.06, 0.25, 0.45])
        self.declare_parameter('max_alpha', 0.75)
        # Only append a new point once the robot has moved this far from
        # the last recorded one -- keeps the trail small without visibly
        # changing its shape.
        self.declare_parameter('min_point_spacing', 0.05)

        self._max_points = self.get_parameter('max_points').value
        self._max_age = self.get_parameter('max_age_sec').value
        self._color_rgb = self.get_parameter('color_rgb').value
        self._max_alpha = self.get_parameter('max_alpha').value
        self._min_spacing = self.get_parameter('min_point_spacing').value
        self._points = []  # list of (Point, stamp_sec_float)
        self._last_xy = None
        self._frame_id = 'map'

        self._pub = self.create_publisher(
            MarkerArray, self.get_parameter('trail_topic').value, 1)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 20)
        self.create_timer(0.2, self._publish)

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_xy is not None:
            dx, dy = x - self._last_xy[0], y - self._last_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self._min_spacing:
                return
        self._last_xy = (x, y)
        self._frame_id = msg.header.frame_id

        now = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        p = Point(x=x, y=y, z=msg.pose.pose.position.z)
        self._points.append((p, now))
        if len(self._points) > self._max_points:
            self._points = self._points[-self._max_points:]

    def _publish(self):
        if not self._points:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        self._points = [(p, t) for p, t in self._points if now - t <= self._max_age]

        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trail'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.pose.orientation.w = 1.0

        r, g, b = self._color_rgb
        for p, t in self._points:
            marker.points.append(p)
            age_fraction = max(0.0, min(1.0, (now - t) / self._max_age))
            alpha = self._max_alpha * (1.0 - age_fraction)
            marker.colors.append(ColorRGBA(r=r, g=g, b=b, a=alpha))

        array = MarkerArray()
        array.markers.append(marker)
        self._pub.publish(array)


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
