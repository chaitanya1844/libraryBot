#!/usr/bin/env python3
"""Core state machine tying RFID reads to Nav2 goals and shelf placement.

Flow per book:
    AWAITING_RFID -> (RfidTag received or manual SortBook call)
    NAVIGATING_TO_SHELF -> Nav2 goToPose to the shelf's approach pose
    LOCATING_SLOT -> take the latest depth-camera shelf-slot reading
    PLACING_BOOK -> teleport the book's Gazebo prop onto the shelf via
                    /libbot/set_entity_state (the simulated equivalent of
                    an arm/gripper placing it -- this repo doesn't model
                    a manipulator, see README)
    RETURNING_TO_STATION -> Nav2 goToPose back to the intake station

Runs the state machine in a background thread so Nav2's blocking-style
BasicNavigator calls don't block the executor that services the RFID
subscription, the SortBook service, and status publishing.
"""
import math
import queue
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from gazebo_msgs.srv import SetEntityState
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from std_srvs.srv import SetBool

from libbot_msgs.msg import RfidTag, ShelfSlot, SortTaskStatus
from libbot_msgs.srv import SortBook


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('sorting_task_manager')

        self.declare_parameter('shelf_map_path', '')
        self.declare_parameter('auto_start', True)

        shelf_map_path = self.get_parameter('shelf_map_path').value
        with open(shelf_map_path) as f:
            shelf_map = yaml.safe_load(f) or {}
        self.shelves = shelf_map.get('shelves', {})
        self.intake = shelf_map.get('intake_station', {'x': 1.0, 'y': 3.5, 'yaw': 1.5708})

        self.job_queue: 'queue.Queue[dict]' = queue.Queue()
        self.books_sorted_total = 0
        self.state = SortTaskStatus.IDLE
        self.current_tag = ''
        self.current_shelf = ''
        self.status_message = 'Waiting for a book'
        self._latest_slot = None
        self._slot_lock = threading.Lock()
        self._enabled = bool(self.get_parameter('auto_start').value)

        self.status_pub = self.create_publisher(SortTaskStatus, '/libbot/sorting/status', 10)
        self.create_subscription(RfidTag, '/libbot/rfid/tag_detected', self._on_rfid_tag, 10)
        self.create_subscription(ShelfSlot, '/libbot/perception/shelf_slot', self._on_shelf_slot, 10)
        self.create_service(SortBook, '/libbot/sorting/sort_book', self._on_sort_book_request)
        self.create_service(SetBool, '/libbot/sorting/set_enabled', self._on_set_enabled)

        self.set_entity_client = self.create_client(SetEntityState, '/libbot/set_entity_state')

        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'Task manager ready: {len(self.shelves)} shelves, intake at '
            f"({self.intake['x']}, {self.intake['y']})")

    # ---------------- ROS callbacks (executor thread) ----------------

    def _on_rfid_tag(self, msg: RfidTag):
        self.job_queue.put({
            'tag_id': msg.tag_id,
            'title': msg.book_title,
            'target_shelf_id': msg.target_shelf_id,
        })

    def _on_shelf_slot(self, msg: ShelfSlot):
        with self._slot_lock:
            self._latest_slot = msg

    def _on_sort_book_request(self, request, response):
        if not request.target_shelf_id and not request.genre:
            response.accepted = False
            response.message = 'Need target_shelf_id or genre'
            return response
        shelf_id = request.target_shelf_id
        if not shelf_id:
            shelf_id = next((sid for sid, s in self.shelves.items()
                              if s.get('genre') == request.genre), '')
        if shelf_id not in self.shelves:
            response.accepted = False
            response.message = f'Unknown shelf_id: {shelf_id}'
            return response
        self.job_queue.put({
            'tag_id': request.tag_id or f'MANUAL-{int(time.time())}',
            'title': request.book_title,
            'target_shelf_id': shelf_id,
        })
        response.accepted = True
        response.message = f'Queued for {shelf_id}'
        return response

    def _on_set_enabled(self, request, response):
        self._enabled = request.data
        response.success = True
        response.message = 'enabled' if self._enabled else 'paused'
        if not self._enabled:
            self.status_message = 'Paused'
        return response

    def _publish_status(self):
        msg = SortTaskStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.tag_id = self.current_tag
        msg.target_shelf_id = self.current_shelf
        msg.message = self.status_message
        msg.books_sorted_total = self.books_sorted_total
        self.status_pub.publish(msg)

    # ---------------- worker thread ----------------

    def run_worker(self, navigator: BasicNavigator):
        while rclpy.ok():
            if not self._enabled:
                time.sleep(0.5)
                continue
            try:
                job = self.job_queue.get(timeout=1.0)
            except queue.Empty:
                self.state = SortTaskStatus.AWAITING_RFID
                self.status_message = 'Waiting for a book'
                continue

            self._run_job(navigator, job)

    def _run_job(self, navigator: BasicNavigator, job: dict):
        tag_id = job['tag_id']
        shelf_id = job['target_shelf_id']
        self.current_tag = tag_id
        self.current_shelf = shelf_id

        shelf = self.shelves.get(shelf_id)
        if shelf is None:
            self.state = SortTaskStatus.ERROR
            self.status_message = f'Unknown shelf_id {shelf_id} for {tag_id}'
            self.get_logger().error(self.status_message)
            return

        self.state = SortTaskStatus.NAVIGATING_TO_SHELF
        self.status_message = f'Taking "{job.get("title", tag_id)}" to {shelf_id}'
        self.get_logger().info(self.status_message)
        if not self._go_to(navigator, shelf['approach_pose']):
            self.state = SortTaskStatus.ERROR
            self.status_message = f'Failed to reach {shelf_id}'
            return

        self.state = SortTaskStatus.LOCATING_SLOT
        self.status_message = f'Scanning {shelf_id} for an open slot'
        slot = self._wait_for_slot(timeout=3.0)

        self.state = SortTaskStatus.PLACING_BOOK
        self.status_message = f'Placing book on {shelf_id}'
        self._place_book(job, shelf, slot)
        self.books_sorted_total += 1

        self.state = SortTaskStatus.RETURNING_TO_STATION
        self.status_message = 'Returning to intake station'
        self._go_to(navigator, self.intake)

        self.current_tag = ''
        self.current_shelf = ''

    def _go_to(self, navigator: BasicNavigator, pose_dict: dict) -> bool:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = float(pose_dict['x'])
        goal.pose.position.y = float(pose_dict['y'])
        goal.pose.orientation = yaw_to_quaternion(float(pose_dict.get('yaw', 0.0)))

        navigator.goToPose(goal)
        while not navigator.isTaskComplete():
            time.sleep(0.2)
        result = navigator.getResult()
        return result == TaskResult.SUCCEEDED

    def _wait_for_slot(self, timeout: float):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._slot_lock:
                if self._latest_slot is not None:
                    return self._latest_slot
            time.sleep(0.1)
        return None

    def _place_book(self, job: dict, shelf: dict, slot):
        """Teleports the book's Gazebo prop onto the shelf. This stands in
        for a manipulator/gripper placement action, which this repo does
        not model (the robot has no arm) -- see README "What's simulated
        vs simplified"."""
        model_name = None
        # book_catalog entries are matched to the RfidTag by the RFID
        # reader; the task manager only knows the tag_id, so it looks the
        # prop name up the same way the reader would have (tag suffix).
        model_name = self._infer_model_name(job['tag_id'])
        if model_name is None or not self.set_entity_client.service_is_ready():
            self.get_logger().warn(
                f"Could not place book prop for {job['tag_id']} "
                '(set_entity_state service unavailable or unknown model)')
            return

        req = SetEntityState.Request()
        req.state.name = model_name
        req.state.pose.position.x = shelf['shelf_pose']['x']
        req.state.pose.position.y = shelf['shelf_pose']['y']
        req.state.pose.position.z = 0.9  # roughly mid-shelf height
        req.state.pose.orientation = yaw_to_quaternion(shelf['shelf_pose'].get('yaw', 0.0))
        req.state.reference_frame = 'world'
        self.set_entity_client.call_async(req)

    def _infer_model_name(self, tag_id: str):
        # BOOK-000001 -> libbot_rfid_book_1, matching book_catalog.yaml's
        # ordering convention. Manual SortBook jobs without a real prop
        # simply skip the visual placement step.
        if not tag_id.startswith('BOOK-'):
            return None
        try:
            n = int(tag_id.split('-')[1])
        except (IndexError, ValueError):
            return None
        return f'libbot_rfid_book_{n}'


def main():
    rclpy.init()
    node = TaskManagerNode()
    navigator = BasicNavigator()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(navigator)

    worker = threading.Thread(target=node.run_worker, args=(navigator,), daemon=True)
    worker.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
