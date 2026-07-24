#!/usr/bin/env python3
"""Simulated RFID reader.

Real RFID hardware isn't modeled at the physics level; instead this node
watches Gazebo's /libbot/model_states for the robot's own pose and for the
poses of the book props listed in book_catalog.yaml, and fires a
libbot_msgs/RfidTag reading whenever the robot comes within read_range of a
book it hasn't already read. This is the "robot picks up a book and its
embedded tag gets scanned" moment that kicks off a sort job.
"""
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header

from libbot_msgs.msg import RfidTag


class RfidReaderNode(Node):

    def __init__(self):
        super().__init__('rfid_reader_node')

        self.declare_parameter('book_catalog_path', '')
        self.declare_parameter('shelf_map_path', '')
        self.declare_parameter('read_range', 0.55)
        self.declare_parameter('robot_model_name', 'libbot')

        catalog_path = self.get_parameter('book_catalog_path').value
        shelf_map_path = self.get_parameter('shelf_map_path').value
        self.read_range = float(self.get_parameter('read_range').value)
        self.robot_model_name = self.get_parameter('robot_model_name').value

        self.books_by_model = {}
        self.genre_to_shelf = {}
        self._load_catalog(catalog_path, shelf_map_path)

        self.already_read = set()

        self.pub = self.create_publisher(RfidTag, '/libbot/rfid/tag_detected', 10)
        self.create_subscription(ModelStates, '/libbot/model_states', self._on_model_states, 10)

        self.get_logger().info(
            f'RFID reader watching {len(self.books_by_model)} book props, '
            f'read_range={self.read_range} m')

    def _load_catalog(self, catalog_path, shelf_map_path):
        import yaml
        if catalog_path:
            with open(catalog_path) as f:
                catalog = yaml.safe_load(f) or {}
            for entry in catalog.get('books', []):
                self.books_by_model[entry['gazebo_model']] = entry
        if shelf_map_path:
            with open(shelf_map_path) as f:
                shelf_map = yaml.safe_load(f) or {}
            for shelf_id, shelf in shelf_map.get('shelves', {}).items():
                self.genre_to_shelf.setdefault(shelf.get('genre'), shelf_id)

    def _on_model_states(self, msg: ModelStates):
        if self.robot_model_name not in msg.name:
            return
        robot_idx = msg.name.index(self.robot_model_name)
        rx = msg.pose[robot_idx].position.x
        ry = msg.pose[robot_idx].position.y

        for model_name, book in self.books_by_model.items():
            tag_id = book['tag_id']
            if tag_id in self.already_read:
                continue
            if model_name not in msg.name:
                continue
            idx = msg.name.index(model_name)
            bx = msg.pose[idx].position.x
            by = msg.pose[idx].position.y
            dist = math.hypot(rx - bx, ry - by)
            if dist <= self.read_range:
                self._fire_tag(book, dist)

    def _fire_tag(self, book, dist):
        tag_id = book['tag_id']
        self.already_read.add(tag_id)

        target_shelf = book.get('target_shelf_id') or self.genre_to_shelf.get(book.get('genre'), '')

        msg = RfidTag()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.tag_id = tag_id
        msg.book_title = book.get('title', '')
        msg.genre = book.get('genre', '')
        msg.target_shelf_id = target_shelf
        msg.rssi = max(0.0, 100.0 - dist * 40.0)
        msg.distance = dist
        self.pub.publish(msg)
        self.get_logger().info(
            f'RFID read: {tag_id} "{msg.book_title}" -> {target_shelf} (dist={dist:.2f}m)')


def main():
    rclpy.init()
    node = RfidReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
