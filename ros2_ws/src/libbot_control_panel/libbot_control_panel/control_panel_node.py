#!/usr/bin/env python3
"""PyQt5 control panel for libraryBot.

Standalone GUI (not an RViz plugin, see README "Control panel" for why):
manual teleop, e-stop, sorting job queue / pause, live task status, pose
readout, and one-click mode switching between mapping and navigation
(each just wraps the corresponding `ros2 launch` command as a subprocess
so you don't need three terminals open to demo the system).

    ros2 run libbot_control_panel control_panel_node.py
"""
import math
import subprocess
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from libbot_msgs.msg import SortTaskStatus
from libbot_msgs.srv import SortBook

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QLineEdit, QComboBox, QTextEdit, QFrame,
)

CMD_VEL_TOPIC = '/libbot/mecanum_drive_controller/reference_unstamped'

STATE_NAMES = {
    SortTaskStatus.IDLE: 'Idle',
    SortTaskStatus.AWAITING_RFID: 'Awaiting book',
    SortTaskStatus.NAVIGATING_TO_SHELF: 'Navigating to shelf',
    SortTaskStatus.LOCATING_SLOT: 'Locating open slot',
    SortTaskStatus.PLACING_BOOK: 'Placing book',
    SortTaskStatus.RETURNING_TO_STATION: 'Returning to station',
    SortTaskStatus.ERROR: 'ERROR',
}

GENRES = ['fiction', 'scifi', 'mystery', 'biography', 'history', 'children',
          'reference', 'poetry']

TELEOP_SPEED_LINEAR = 0.4
TELEOP_SPEED_ANGULAR = 0.6


class ControlPanelNode(Node):

    def __init__(self):
        super().__init__('libbot_control_panel')
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(SortTaskStatus, '/libbot/sorting/status', self._on_status, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)

        self.sort_book_client = self.create_client(SortBook, '/libbot/sorting/sort_book')
        self.set_enabled_client = self.create_client(SetBool, '/libbot/sorting/set_enabled')

        self.latest_status = None
        self.latest_pose = None
        self._estop_active = False

    def _on_status(self, msg: SortTaskStatus):
        self.latest_status = msg

    def _on_odom(self, msg: Odometry):
        self.latest_pose = msg.pose.pose

    def publish_cmd(self, vx: float, vy: float, wz: float):
        if self._estop_active:
            return
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = wz
        self.cmd_pub.publish(t)

    def stop(self):
        t = Twist()
        self.cmd_pub.publish(t)

    def set_estop(self, active: bool):
        self._estop_active = active
        if active:
            for _ in range(3):
                self.cmd_pub.publish(Twist())

    def send_sort_book(self, tag_id, title, genre, shelf_id, callback):
        if not self.sort_book_client.service_is_ready():
            callback(False, 'sorting_task_manager not running')
            return
        req = SortBook.Request()
        req.tag_id = tag_id
        req.book_title = title
        req.genre = genre
        req.target_shelf_id = shelf_id
        future = self.sort_book_client.call_async(req)
        future.add_done_callback(
            lambda f: callback(f.result().accepted, f.result().message))

    def set_sorting_enabled(self, enabled: bool):
        if not self.set_enabled_client.service_is_ready():
            return
        req = SetBool.Request()
        req.data = enabled
        self.set_enabled_client.call_async(req)


class MainWindow(QMainWindow):

    def __init__(self, node: ControlPanelNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('libraryBot Control Panel')
        self.resize(560, 640)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        layout.addWidget(self._build_status_box())
        layout.addWidget(self._build_teleop_box())
        layout.addWidget(self._build_sorting_box())
        layout.addWidget(self._build_mode_box())
        layout.addWidget(self._build_log_box())

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(50)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_status)
        self.ui_timer.start(300)

    # ---------------- ROS pump ----------------

    def _spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def _log(self, text: str):
        self.log.append(text)

    # ---------------- status box ----------------

    def _build_status_box(self):
        box = QGroupBox('Status')
        grid = QGridLayout(box)
        self.state_label = QLabel('--')
        self.tag_label = QLabel('--')
        self.shelf_label = QLabel('--')
        self.message_label = QLabel('--')
        self.count_label = QLabel('0')
        self.pose_label = QLabel('--')

        for row, (name, widget) in enumerate([
            ('State', self.state_label),
            ('Current tag', self.tag_label),
            ('Target shelf', self.shelf_label),
            ('Message', self.message_label),
            ('Books sorted', self.count_label),
            ('Pose (map, x/y/yaw)', self.pose_label),
        ]):
            grid.addWidget(QLabel(f'<b>{name}:</b>'), row, 0)
            grid.addWidget(widget, row, 1)
        return box

    def _refresh_status(self):
        s = self.node.latest_status
        if s is not None:
            self.state_label.setText(STATE_NAMES.get(s.state, str(s.state)))
            self.tag_label.setText(s.tag_id or '--')
            self.shelf_label.setText(s.target_shelf_id or '--')
            self.message_label.setText(s.message)
            self.count_label.setText(str(s.books_sorted_total))
        p = self.node.latest_pose
        if p is not None:
            yaw = math.degrees(2 * math.atan2(p.orientation.z, p.orientation.w))
            self.pose_label.setText(f'{p.position.x:.2f}, {p.position.y:.2f}, {yaw:.1f} deg')

    # ---------------- teleop box ----------------

    def _build_teleop_box(self):
        box = QGroupBox('Manual Drive (hold button; releases stop the robot)')
        grid = QGridLayout(box)

        def make(label, vx, vy, wz):
            btn = QPushButton(label)
            btn.pressed.connect(lambda: self.node.publish_cmd(vx, vy, wz))
            btn.released.connect(self.node.stop)
            return btn

        s, a = TELEOP_SPEED_LINEAR, TELEOP_SPEED_ANGULAR
        grid.addWidget(make('Rotate CCW', 0, 0, a), 0, 0)
        grid.addWidget(make('Forward', s, 0, 0), 0, 1)
        grid.addWidget(make('Rotate CW', 0, 0, -a), 0, 2)
        grid.addWidget(make('Strafe Left', 0, s, 0), 1, 0)
        grid.addWidget(make('Backward', -s, 0, 0), 1, 1)
        grid.addWidget(make('Strafe Right', 0, -s, 0), 1, 2)

        estop_btn = QPushButton('E-STOP')
        estop_btn.setStyleSheet('background-color: #c0392b; color: white; font-weight: bold;')
        estop_btn.setCheckable(True)
        estop_btn.toggled.connect(self._on_estop_toggled)
        grid.addWidget(estop_btn, 2, 0, 1, 3)
        self.estop_btn = estop_btn
        return box

    def _on_estop_toggled(self, checked):
        self.node.set_estop(checked)
        self.estop_btn.setText('E-STOP ENGAGED (click to release)' if checked else 'E-STOP')
        self._log('E-stop engaged' if checked else 'E-stop released')

    # ---------------- sorting box ----------------

    def _build_sorting_box(self):
        box = QGroupBox('Sorting')
        v = QVBoxLayout(box)

        toggle_row = QHBoxLayout()
        start_btn = QPushButton('Start / Resume Sorting')
        start_btn.clicked.connect(lambda: (self.node.set_sorting_enabled(True), self._log('Sorting resumed')))
        pause_btn = QPushButton('Pause Sorting')
        pause_btn.clicked.connect(lambda: (self.node.set_sorting_enabled(False), self._log('Sorting paused')))
        toggle_row.addWidget(start_btn)
        toggle_row.addWidget(pause_btn)
        v.addLayout(toggle_row)

        form = QHBoxLayout()
        self.tag_input = QLineEdit()
        self.tag_input.setPlaceholderText('tag_id (blank = auto)')
        self.title_input = QLineEdit()
        self.title_input.setPlaceholderText('title (optional)')
        self.genre_combo = QComboBox()
        self.genre_combo.addItems(GENRES)
        send_btn = QPushButton('Queue manual sort')
        send_btn.clicked.connect(self._on_manual_sort)
        form.addWidget(self.tag_input)
        form.addWidget(self.title_input)
        form.addWidget(self.genre_combo)
        form.addWidget(send_btn)
        v.addLayout(form)
        return box

    def _on_manual_sort(self):
        tag = self.tag_input.text().strip()
        title = self.title_input.text().strip()
        genre = self.genre_combo.currentText()

        def done(accepted, message):
            self._log(f'Sort request: {"OK" if accepted else "REJECTED"} - {message}')

        self.node.send_sort_book(tag, title, genre, '', done)
        self._log(f'Queuing "{title or tag or "book"}" ({genre})...')

    # ---------------- mode box ----------------

    def _build_mode_box(self):
        box = QGroupBox('Mode (do these in order, each stops the previous one)')
        v = QVBoxLayout(box)
        row1 = QHBoxLayout()
        mapping_btn = QPushButton('1. Start Mapping')
        mapping_btn.clicked.connect(self._start_mapping)
        save_btn = QPushButton('Save Map')
        save_btn.clicked.connect(self._save_map)
        row1.addWidget(mapping_btn)
        row1.addWidget(save_btn)
        v.addLayout(row1)

        loc_btn = QPushButton('2. Start Localization')
        loc_btn.clicked.connect(self._start_localization)
        v.addWidget(loc_btn)

        nav_btn = QPushButton('3. Start Navigation')
        nav_btn.clicked.connect(self._start_navigation)
        v.addWidget(nav_btn)

        help_label = QLabel(
            '1. Start Mapping, drive around until the map looks complete, then Save Map.\n'
            '2. Start Localization to load the saved map and locate the robot (AMCL).\n'
            '3. Start Navigation to bring up Nav2 + RFID + sorting.')
        help_label.setWordWrap(True)
        v.addWidget(help_label)
        return box

    def _switch_mode_script(self):
        return '/home/sychic/libraryBot/ros2_ws/install/libbot_bringup/lib/libbot_bringup/switch_mode.sh'

    def _run_bg(self, cmd, label):
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._log(f'Launched: {label}')
        except Exception as exc:  # noqa: BLE001
            self._log(f'Failed to launch {label}: {exc}')

    def _start_mapping(self):
        self._run_bg(['bash', self._switch_mode_script(), 'mapping'], 'mapping (SLAM)')

    def _start_localization(self):
        map_path = '/home/sychic/libraryBot/ros2_ws/src/libbot_mapping/maps/library.yaml'
        self._run_bg(['bash', self._switch_mode_script(), 'localization', map_path],
                      'localization (AMCL)')

    def _start_navigation(self):
        self._run_bg(['bash', self._switch_mode_script(), 'navigation'],
                      'navigation (Nav2 + sorting)')

    def _save_map(self):
        self._run_bg(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                       '-f', '/home/sychic/libraryBot/ros2_ws/src/libbot_mapping/maps/library'],
                      'map save')

    # ---------------- log box ----------------

    def _build_log_box(self):
        box = QGroupBox('Log')
        v = QVBoxLayout(box)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        v.addWidget(self.log)
        return box


def main():
    rclpy.init()
    node = ControlPanelNode()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()

    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
