#!/usr/bin/env python3
"""PyQt5 control panel for libraryBot.

Standalone GUI (not an RViz plugin, see README "Control panel" for why):
e-stop, live pose readout, and one-click mode switching between
mapping/localization/navigation (each just wraps the corresponding
`ros2 launch` command as a subprocess so you don't need three terminals
open to demo the system). Manual driving is the Xbox controller
(libbot_control/teleop_joy.launch.py); giving Nav2 a goal is RViz's own
"2D Pose Estimate" / "Nav2 Goal" toolbar tools.

    ros2 run libbot_control_panel control_panel_node.py
"""
import math
import subprocess
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from controller_manager_msgs.srv import SwitchController

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout,
    QPushButton, QLabel, QGroupBox, QTextEdit,
)

CMD_VEL_TOPIC = '/libbot/mecanum_drive_controller/reference_unstamped'


class ControlPanelNode(Node):

    def __init__(self):
        super().__init__('libbot_control_panel')
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.switch_controller_client = self.create_client(
            SwitchController, '/libbot/controller_manager/switch_controller')

        self.latest_pose = None

    def _on_odom(self, msg: Odometry):
        self.latest_pose = msg.pose.pose

    def set_estop(self, active: bool):
        # A one-shot zero Twist isn't enough once the Xbox controller (or
        # Nav2) is publishing to the same topic independently -- actually
        # deactivate the mecanum_drive_controller so the robot physically
        # cannot move no matter who is publishing Twists.
        self.cmd_pub.publish(Twist())
        if not self.switch_controller_client.service_is_ready():
            return
        req = SwitchController.Request()
        if active:
            req.deactivate_controllers = ['mecanum_drive_controller']
        else:
            req.activate_controllers = ['mecanum_drive_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        self.switch_controller_client.call_async(req)


class MainWindow(QMainWindow):

    def __init__(self, node: ControlPanelNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('libraryBot Control Panel')
        self.resize(480, 420)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        layout.addWidget(self._build_status_box())
        layout.addWidget(self._build_estop_box())
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
        self.pose_label = QLabel('--')
        grid.addWidget(QLabel('<b>Pose (map, x/y/yaw):</b>'), 0, 0)
        grid.addWidget(self.pose_label, 0, 1)
        return box

    def _refresh_status(self):
        p = self.node.latest_pose
        if p is not None:
            yaw = math.degrees(2 * math.atan2(p.orientation.z, p.orientation.w))
            self.pose_label.setText(f'{p.position.x:.2f}, {p.position.y:.2f}, {yaw:.1f} deg')

    # ---------------- e-stop box ----------------

    def _build_estop_box(self):
        box = QGroupBox('E-STOP')
        v = QVBoxLayout(box)
        estop_btn = QPushButton('E-STOP')
        estop_btn.setStyleSheet('background-color: #c0392b; color: white; font-weight: bold;')
        estop_btn.setCheckable(True)
        estop_btn.setMinimumHeight(44)
        estop_btn.toggled.connect(self._on_estop_toggled)
        v.addWidget(estop_btn)
        self.estop_btn = estop_btn
        return box

    def _on_estop_toggled(self, checked):
        self.node.set_estop(checked)
        self.estop_btn.setText('E-STOP ENGAGED (click to release)' if checked else 'E-STOP')
        self._log('E-stop engaged: drive controller deactivated' if checked
                   else 'E-stop released: drive controller reactivated')

    # ---------------- mode box ----------------

    def _build_mode_box(self):
        box = QGroupBox('Mode (do these in order, each stops the previous one)')
        v = QVBoxLayout(box)
        row1 = QGridLayout()
        mapping_btn = QPushButton('1. Start Mapping')
        mapping_btn.clicked.connect(self._start_mapping)
        save_btn = QPushButton('Save Map')
        save_btn.clicked.connect(self._save_map)
        row1.addWidget(mapping_btn, 0, 0)
        row1.addWidget(save_btn, 0, 1)
        v.addLayout(row1)

        loc_btn = QPushButton('2. Start Localization')
        loc_btn.clicked.connect(self._start_localization)
        v.addWidget(loc_btn)

        nav_btn = QPushButton('3. Start Navigation')
        nav_btn.clicked.connect(self._start_navigation)
        v.addWidget(nav_btn)

        help_label = QLabel(
            '1. Start Mapping, drive around (Xbox controller) until the map '
            'looks complete, then Save Map.\n'
            '2. Start Localization to load the saved map and locate the '
            'robot (AMCL).\n'
            '3. Start Navigation to bring up Nav2, then use RViz\'s '
            '"2D Pose Estimate" / "Nav2 Goal" tools.')
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
        self._run_bg(['bash', self._switch_mode_script(), 'navigation'], 'navigation (Nav2)')

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
