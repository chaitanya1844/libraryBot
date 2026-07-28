#include "libbot_rviz_plugins/libbot_control_panel.hpp"

#include <chrono>
#include <cmath>

namespace libbot_rviz_plugins
{

namespace
{
constexpr const char * kCmdVelTopic = "/libbot/mecanum_drive_controller/reference_unstamped";
}  // namespace

LibbotControlPanel::LibbotControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  // Everything lives in a scrollable inner widget rather than directly on
  // the panel: this dock has a lot of stacked controls, and without a
  // scroll area Qt/the window manager treat that stack's natural height
  // as a hard MINIMUM for the whole RViz window (observed forcing a
  // ~3000px tall window on a 1440px screen, ignoring any configured
  // Window Geometry). Scrolling keeps the dock itself small regardless
  // of how many controls it holds.
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  auto * content = new QWidget();
  auto * root = new QVBoxLayout(content);

  // ---------------- status ----------------
  auto * status_box = new QGroupBox("Status");
  auto * status_grid = new QGridLayout(status_box);
  pose_label_ = new QLabel("--");
  status_grid->addWidget(new QLabel("<b>Pose:</b>"), 0, 0);
  status_grid->addWidget(pose_label_, 0, 1);
  root->addWidget(status_box);

  // ---------------- e-stop ----------------
  // Manual driving is done with the Xbox controller (see
  // `ros2 launch libbot_control teleop_joy.launch.py`, hold LB + move
  // sticks) rather than on-screen buttons -- this panel keeps only the
  // E-STOP, which works regardless of what's driving the robot.
  auto * estop_box = new QGroupBox("E-STOP");
  auto * estop_v = new QVBoxLayout(estop_box);
  estop_button_ = new QPushButton("E-STOP");
  estop_button_->setCheckable(true);
  estop_button_->setMinimumHeight(44);
  estop_button_->setStyleSheet(
    "QPushButton { background-color: #c0392b; color: white; font-weight: bold; }"
    "QPushButton:checked { background-color: #7b241c; }");
  connect(estop_button_, &QPushButton::toggled, this, &LibbotControlPanel::onEStopToggled);
  estop_v->addWidget(estop_button_);
  auto * reset_pos_btn = new QPushButton("Reset Position");
  connect(reset_pos_btn, &QPushButton::clicked, this, &LibbotControlPanel::onResetPosition);
  estop_v->addWidget(reset_pos_btn);
  root->addWidget(estop_box);

  // ---------------- teleop override ----------------
  // Nav2's controller_server only publishes cmd_vel while it has an
  // active goal -- canceling that goal (which this toggle does) leaves
  // the topic free for these buttons with nothing to fight over, no
  // need to touch ros2_control like E-STOP does. Toggle off to hand
  // control back to Nav2 (send it a new goal to resume).
  auto * teleop_box = new QGroupBox("Teleop Override");
  auto * teleop_v = new QVBoxLayout(teleop_box);
  teleop_button_ = new QPushButton("Teleop Override");
  teleop_button_->setCheckable(true);
  teleop_button_->setMinimumHeight(40);
  teleop_button_->setStyleSheet(
    "QPushButton { background-color: #b9770e; color: white; font-weight: bold; }"
    "QPushButton:checked { background-color: #7d5109; }");
  connect(teleop_button_, &QPushButton::toggled, this, &LibbotControlPanel::onTeleopToggled);
  teleop_v->addWidget(teleop_button_);

  auto * teleop_grid = new QGridLayout();
  teleop_rotate_ccw_ = new QPushButton("↺ CCW");
  teleop_forward_ = new QPushButton("↑ Forward");
  teleop_rotate_cw_ = new QPushButton("↻ CW");
  teleop_strafe_left_ = new QPushButton("← Strafe");
  teleop_stop_ = new QPushButton("STOP");
  teleop_strafe_right_ = new QPushButton("Strafe →");
  teleop_back_ = new QPushButton("↓ Back");
  teleop_grid->addWidget(teleop_rotate_ccw_, 0, 0);
  teleop_grid->addWidget(teleop_forward_, 0, 1);
  teleop_grid->addWidget(teleop_rotate_cw_, 0, 2);
  teleop_grid->addWidget(teleop_strafe_left_, 1, 0);
  teleop_grid->addWidget(teleop_stop_, 1, 1);
  teleop_grid->addWidget(teleop_strafe_right_, 1, 2);
  teleop_grid->addWidget(teleop_back_, 2, 1);
  // Press-and-hold, not click-and-forget: `pressed()`/`released()` (mouse
  // down/up) rather than `clicked()` (mouse up only). Only publishing
  // while physically held means this stays silent on the shared cmd_vel
  // topic the rest of the time, so it doesn't fight the Xbox joystick
  // teleop (teleop_joy.launch.py) publishing to the same topic -- both
  // can coexist as long as you're not holding a panel button and the
  // stick at the same instant.
  connect(teleop_forward_, &QPushButton::pressed, this, &LibbotControlPanel::onTeleopForward);
  connect(teleop_back_, &QPushButton::pressed, this, &LibbotControlPanel::onTeleopBack);
  connect(teleop_strafe_left_, &QPushButton::pressed, this, &LibbotControlPanel::onTeleopStrafeLeft);
  connect(teleop_strafe_right_, &QPushButton::pressed, this, &LibbotControlPanel::onTeleopStrafeRight);
  connect(teleop_rotate_cw_, &QPushButton::pressed, this, &LibbotControlPanel::onTeleopRotateCw);
  connect(teleop_rotate_ccw_, &QPushButton::pressed, this, &LibbotControlPanel::onTeleopRotateCcw);
  for (auto * btn : {teleop_forward_, teleop_back_, teleop_strafe_left_, teleop_strafe_right_,
         teleop_rotate_cw_, teleop_rotate_ccw_}) {
    connect(btn, &QPushButton::released, this, &LibbotControlPanel::onTeleopReleased);
  }
  connect(teleop_stop_, &QPushButton::clicked, this, &LibbotControlPanel::onTeleopStop);
  teleop_v->addLayout(teleop_grid);
  root->addWidget(teleop_box);
  setTeleopButtonsEnabled(false);

  // ---------------- arm ----------------
  auto * arm_box = new QGroupBox("Arm");
  auto * arm_grid = new QGridLayout(arm_box);
  static const struct { const char * name; const char * label; double lower; double upper; } kArmJoints[4] = {
    {"shoulder_pan_joint", "Pan", -2.0, 2.0},
    {"shoulder_lift_joint", "Lift", -1.6, 1.2},
    {"elbow_joint", "Elbow", -2.2, 1.8},
    {"wrist_pitch_joint", "Wrist", -1.6, 1.6},
  };
  for (int i = 0; i < 4; ++i) {
    auto * slider = new QSlider(Qt::Horizontal);
    slider->setRange(static_cast<int>(kArmJoints[i].lower * 1000), static_cast<int>(kArmJoints[i].upper * 1000));
    slider->setValue(0);
    auto * value_label = new QLabel("0.00");
    value_label->setMinimumWidth(45);
    connect(slider, &QSlider::valueChanged, this, [value_label](int v) {
      value_label->setText(QString::number(v / 1000.0, 'f', 2));
    });
    arm_grid->addWidget(new QLabel(kArmJoints[i].label), i, 0);
    arm_grid->addWidget(slider, i, 1);
    arm_grid->addWidget(value_label, i, 2);
    arm_sliders_[i] = slider;
  }
  auto * arm_move_btn = new QPushButton("Move Arm");
  auto * arm_reset_btn = new QPushButton("Reset to Default");
  connect(arm_move_btn, &QPushButton::clicked, this, &LibbotControlPanel::onMoveArm);
  connect(arm_reset_btn, &QPushButton::clicked, this, &LibbotControlPanel::onResetArm);
  arm_grid->addWidget(arm_move_btn, 4, 0, 1, 3);
  arm_grid->addWidget(arm_reset_btn, 5, 0, 1, 3);
  root->addWidget(arm_box);

  // ---------------- mode: Map -> Localize -> Navigate ----------------
  auto * mode_box = new QGroupBox("Mode");
  auto * mode_grid = new QGridLayout(mode_box);

  map_combo_ = new QComboBox();
  auto * refresh_maps_btn = new QPushButton("Refresh");
  auto * delete_map_btn = new QPushButton("Delete");
  connect(refresh_maps_btn, &QPushButton::clicked, this, &LibbotControlPanel::onRefreshMaps);
  connect(delete_map_btn, &QPushButton::clicked, this, &LibbotControlPanel::onDeleteMap);
  mode_grid->addWidget(new QLabel("Map:"), 0, 0);
  mode_grid->addWidget(map_combo_, 0, 1);
  mode_grid->addWidget(refresh_maps_btn, 0, 2);
  mode_grid->addWidget(delete_map_btn, 0, 3);

  start_mapping_button_ = new QPushButton("Start Mapping");
  stop_mapping_button_ = new QPushButton("Stop");
  save_map_button_ = new QPushButton("Save Map As...");
  connect(start_mapping_button_, &QPushButton::clicked, this, &LibbotControlPanel::onStartMapping);
  connect(stop_mapping_button_, &QPushButton::clicked, this, &LibbotControlPanel::onStopMapping);
  connect(save_map_button_, &QPushButton::clicked, this, &LibbotControlPanel::onSaveMap);
  mode_grid->addWidget(start_mapping_button_, 1, 0, 1, 2);
  mode_grid->addWidget(stop_mapping_button_, 1, 2);
  mode_grid->addWidget(save_map_button_, 1, 3);

  start_localization_button_ = new QPushButton("Start Localization");
  stop_localization_button_ = new QPushButton("Stop");
  connect(start_localization_button_, &QPushButton::clicked, this, &LibbotControlPanel::onStartLocalization);
  connect(stop_localization_button_, &QPushButton::clicked, this, &LibbotControlPanel::onStopLocalization);
  mode_grid->addWidget(start_localization_button_, 2, 0, 1, 3);
  mode_grid->addWidget(stop_localization_button_, 2, 3);

  start_navigation_button_ = new QPushButton("Start Navigation");
  stop_navigation_button_ = new QPushButton("Stop");
  connect(start_navigation_button_, &QPushButton::clicked, this, &LibbotControlPanel::onStartNavigation);
  connect(stop_navigation_button_, &QPushButton::clicked, this, &LibbotControlPanel::onStopNavigation);
  mode_grid->addWidget(start_navigation_button_, 3, 0, 1, 3);
  mode_grid->addWidget(stop_navigation_button_, 3, 3);

  // RViz's own "Nav2 Goal" toolbar tool (click-and-drag on the 3D view)
  // didn't register any click in a remote-desktop test environment --
  // same class of input-routing quirk already seen with the standalone
  // PyQt5 panel's buttons there, just hitting RViz's 3D viewport instead
  // this time. Goal X/Y fields here send the exact same NavigateToPose
  // action goal without needing that 3D-view drag gesture at all.
  goal_x_ = new QDoubleSpinBox();
  goal_x_->setRange(-100.0, 100.0);
  goal_x_->setDecimals(2);
  goal_y_ = new QDoubleSpinBox();
  goal_y_->setRange(-100.0, 100.0);
  goal_y_->setDecimals(2);
  auto * send_goal_btn = new QPushButton("Send Goal");
  connect(send_goal_btn, &QPushButton::clicked, this, &LibbotControlPanel::onSendGoal);
  mode_grid->addWidget(new QLabel("Goal X/Y:"), 4, 0);
  mode_grid->addWidget(goal_x_, 4, 1);
  mode_grid->addWidget(goal_y_, 4, 2);
  mode_grid->addWidget(send_goal_btn, 4, 3);

  auto * stop_all_btn = new QPushButton("STOP EVERYTHING");
  stop_all_btn->setStyleSheet("QPushButton { background-color: #922b21; color: white; font-weight: bold; }");
  connect(stop_all_btn, &QPushButton::clicked, this, &LibbotControlPanel::onStopAll);
  mode_grid->addWidget(stop_all_btn, 5, 0, 1, 4);

  root->addWidget(mode_box);
  updateModeButtons();

  // ---------------- log ----------------
  auto * log_box = new QGroupBox("Log");
  auto * log_v = new QVBoxLayout(log_box);
  log_view_ = new QTextEdit();
  log_view_->setReadOnly(true);
  log_view_->setMaximumHeight(120);
  log_v->addWidget(log_view_);
  root->addWidget(log_box);

  content->setLayout(root);
  scroll->setWidget(content);
  outer->addWidget(scroll);
  setLayout(outer);
}

LibbotControlPanel::~LibbotControlPanel()
{
}

void LibbotControlPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("libbot_control_panel_rviz");

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, 10);

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_odom_ = msg;
    });

  switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
    "/libbot/controller_manager/switch_controller");

  nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node_, "/navigate_to_pose");

  map_save_client_ = node_->create_client<std_srvs::srv::Trigger>("/map_save");

  set_entity_state_client_ = node_->create_client<gazebo_msgs::srv::SetEntityState>(
    "/libbot/set_entity_state");

  arm_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node_, "/libbot/arm_controller/follow_joint_trajectory");

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &LibbotControlPanel::onSpinTimer);
  spin_timer_->start(50);

  ui_timer_ = new QTimer(this);
  connect(ui_timer_, &QTimer::timeout, this, &LibbotControlPanel::onUiTimer);
  ui_timer_->start(300);

  onRefreshMaps();
  log("Panel initialized");
}

void LibbotControlPanel::onSpinTimer()
{
  if (node_) {
    rclcpp::spin_some(node_);
  }
  // Only republished while a direction button is physically held down
  // (see the pressed()/released() wiring in the constructor) -- silent
  // the rest of the time so it doesn't fight the Xbox joystick teleop
  // publishing to the same topic.
  if (teleop_active_ && teleop_key_held_) {
    cmd_pub_->publish(teleop_cmd_);
  }
}

void LibbotControlPanel::onUiTimer()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (latest_odom_) {
    const auto & p = latest_odom_->pose.pose;
    double yaw = std::atan2(2.0 * p.orientation.z * p.orientation.w, 1.0 - 2.0 * p.orientation.z * p.orientation.z);
    pose_label_->setText(
      QString("%1, %2, %3 deg").arg(p.position.x, 0, 'f', 2).arg(p.position.y, 0, 'f', 2)
      .arg(yaw * 180.0 / M_PI, 0, 'f', 1));
  }
}

void LibbotControlPanel::onEStopToggled(bool checked)
{
  estop_active_ = checked;
  estop_button_->setText(checked ? "E-STOP ENGAGED (click to release)" : "E-STOP");
  setEStop(checked);
}

void LibbotControlPanel::setEStop(bool engaged)
{
  // A one-shot zero Twist isn't enough once the Xbox controller (or Nav2)
  // is publishing to the same topic independently -- the next stick event
  // or plan cycle would immediately overwrite it. Instead, actually
  // deactivate the mecanum_drive_controller in ros2_control: with no
  // active controller consuming commands, the robot physically cannot
  // move no matter who is publishing Twists. Reactivating on release
  // restores normal operation.
  geometry_msgs::msg::Twist zero;
  cmd_pub_->publish(zero);

  if (!switch_controller_client_->service_is_ready()) {
    log(engaged ? "E-stop engaged (zero Twist only)" : "E-stop released");
    return;
  }
  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  if (engaged) {
    req->deactivate_controllers = {"mecanum_drive_controller"};
  } else {
    req->activate_controllers = {"mecanum_drive_controller"};
  }
  req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  switch_controller_client_->async_send_request(req);
  log(engaged ? "E-stop engaged" : "E-stop released");
}

namespace
{
// Must match libbot_gazebo/launch/spawn_robot.launch.py's default x/y/z/yaw.
constexpr double kSpawnX = 1.0;
constexpr double kSpawnY = 3.3;
constexpr double kSpawnZ = 0.05;
constexpr double kSpawnYaw = -1.5708;
}  // namespace

void LibbotControlPanel::onResetPosition()
{
  if (!set_entity_state_client_->service_is_ready()) {
    log("Reset failed: /libbot/set_entity_state not available");
    return;
  }
  auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  req->state.name = "libbot";
  req->state.reference_frame = "world";
  req->state.pose.position.x = kSpawnX;
  req->state.pose.position.y = kSpawnY;
  req->state.pose.position.z = kSpawnZ;
  req->state.pose.orientation.z = std::sin(kSpawnYaw / 2.0);
  req->state.pose.orientation.w = std::cos(kSpawnYaw / 2.0);
  // Zeroed explicitly (default-constructed Twist is already all-zero, but
  // a topple/collision can leave real residual velocity that would carry
  // straight through the teleport otherwise).
  req->state.twist = geometry_msgs::msg::Twist();
  set_entity_state_client_->async_send_request(
    req, [this](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture future) {
      auto resp = future.get();
      QMetaObject::invokeMethod(this, [this, ok = resp->success]() {
        log(ok ? "Position reset" : "Reset failed");
      });
    });
}

void LibbotControlPanel::onTeleopToggled(bool checked)
{
  teleop_active_ = checked;
  teleop_button_->setText(checked ? "Teleop Override ENGAGED (click to release)" : "Teleop Override");
  setTeleopButtonsEnabled(checked);

  if (checked) {
    if (nav_action_client_ && nav_action_client_->action_server_is_ready()) {
      nav_action_client_->async_cancel_all_goals();
    }
    log("Teleop override engaged");
  } else {
    teleop_key_held_ = false;
    teleop_cmd_ = geometry_msgs::msg::Twist();
    cmd_pub_->publish(teleop_cmd_);
    log("Teleop override released");
  }
}

void LibbotControlPanel::setTeleopButtonsEnabled(bool enabled)
{
  for (auto * btn : {teleop_forward_, teleop_back_, teleop_strafe_left_, teleop_strafe_right_,
         teleop_rotate_cw_, teleop_rotate_ccw_, teleop_stop_}) {
    if (btn) {
      btn->setEnabled(enabled);
    }
  }
}

namespace
{
constexpr double kTeleopLinear = 0.3;
constexpr double kTeleopAngular = 0.5;
}  // namespace

void LibbotControlPanel::onTeleopForward()
{
  teleop_cmd_ = geometry_msgs::msg::Twist();
  teleop_cmd_.linear.x = kTeleopLinear;
  teleop_key_held_ = true;
}

void LibbotControlPanel::onTeleopBack()
{
  teleop_cmd_ = geometry_msgs::msg::Twist();
  teleop_cmd_.linear.x = -kTeleopLinear;
  teleop_key_held_ = true;
}

void LibbotControlPanel::onTeleopStrafeLeft()
{
  teleop_cmd_ = geometry_msgs::msg::Twist();
  teleop_cmd_.linear.y = kTeleopLinear;
  teleop_key_held_ = true;
}

void LibbotControlPanel::onTeleopStrafeRight()
{
  teleop_cmd_ = geometry_msgs::msg::Twist();
  teleop_cmd_.linear.y = -kTeleopLinear;
  teleop_key_held_ = true;
}

void LibbotControlPanel::onTeleopRotateCw()
{
  teleop_cmd_ = geometry_msgs::msg::Twist();
  teleop_cmd_.angular.z = -kTeleopAngular;
  teleop_key_held_ = true;
}

void LibbotControlPanel::onTeleopRotateCcw()
{
  teleop_cmd_ = geometry_msgs::msg::Twist();
  teleop_cmd_.angular.z = kTeleopAngular;
  teleop_key_held_ = true;
}

void LibbotControlPanel::onTeleopReleased()
{
  teleop_key_held_ = false;
  teleop_cmd_ = geometry_msgs::msg::Twist();
  cmd_pub_->publish(teleop_cmd_);
}

void LibbotControlPanel::onTeleopStop()
{
  teleop_key_held_ = false;
  teleop_cmd_ = geometry_msgs::msg::Twist();
  cmd_pub_->publish(teleop_cmd_);
}

namespace
{
QString switchModeScript()
{
  return "bash /home/sychic/libraryBot/ros2_ws/install/libbot_bringup/lib/"
         "libbot_bringup/switch_mode.sh";
}
}  // namespace

void LibbotControlPanel::onMoveArm()
{
  static const char * kJointNames[4] = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_pitch_joint"};

  if (!arm_action_client_->action_server_is_ready()) {
    log("Move Arm failed: arm_controller not available");
    return;
  }
  control_msgs::action::FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names.assign(kJointNames, kJointNames + 4);
  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (int i = 0; i < 4; ++i) {
    point.positions.push_back(arm_sliders_[i]->value() / 1000.0);
  }
  point.time_from_start = rclcpp::Duration::from_seconds(1.5);
  goal.trajectory.points.push_back(point);
  arm_action_client_->async_send_goal(goal);
  log("Arm moving...");
}

void LibbotControlPanel::onResetArm()
{
  // Same stow pose as arm_stow_node.py's POSE_STOW.
  static const double kDefaultPose[4] = {0.0, -1.4, 1.6, 0.4};
  for (int i = 0; i < 4; ++i) {
    arm_sliders_[i]->setValue(static_cast<int>(kDefaultPose[i] * 1000));
  }
  onMoveArm();
}

void LibbotControlPanel::onStartMapping()
{
  QProcess::startDetached("bash", {"-c", switchModeScript() + " mapping"});
  mapping_active_ = true;
  localization_active_ = false;
  navigation_active_ = false;
  updateModeButtons();
  resetDisplays();
  log("Mapping started");
}

void LibbotControlPanel::onStartLocalization()
{
  QString map_path = selectedMapYamlPath();
  if (map_path.isEmpty()) {
    log("No map selected");
    return;
  }
  QProcess::startDetached("bash", {"-c", switchModeScript() + " localization \"" + map_path + "\""});
  mapping_active_ = false;
  localization_active_ = true;
  navigation_active_ = false;
  updateModeButtons();
  resetDisplays();
  log("Localization started against '" + map_combo_->currentText() +
      "' -- use RViz's \"2D Pose Estimate\" to seed it");
}

void LibbotControlPanel::onStartNavigation()
{
  // Stow first, *then* start driving -- an arm left extended (e.g. from
  // the sliders above) clipping a wall mid-drive is exactly how the
  // robot toppled here. arm_stow_node.py only ever stows once, a few
  // seconds after simulation.launch.py starts; nothing re-stows it
  // before a later Start Navigation if the arm moved since then.
  onResetArm();
  QTimer::singleShot(1600, this, [this]() {
    QProcess::startDetached("bash", {"-c", switchModeScript() + " navigation"});
    navigation_active_ = true;
    updateModeButtons();
    log("Navigation started");
  });
  log("Stowing arm before navigation...");
}

void LibbotControlPanel::onSendGoal()
{
  if (!nav_action_client_ || !nav_action_client_->action_server_is_ready()) {
    log("Send Goal failed: navigate_to_pose action server not available -- Start Navigation first");
    return;
  }
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = goal_x_->value();
  goal.pose.pose.position.y = goal_y_->value();
  goal.pose.pose.orientation.w = 1.0;
  nav_action_client_->async_send_goal(goal);
  log(QString("Goal sent: (%1, %2)").arg(goal_x_->value(), 0, 'f', 2).arg(goal_y_->value(), 0, 'f', 2));
}

void LibbotControlPanel::onStopMapping()
{
  QProcess::startDetached("bash", {"-c", switchModeScript() + " stop"});
  mapping_active_ = false;
  updateModeButtons();
  resetDisplays();
  log("Mapping stopped");
}

void LibbotControlPanel::onStopLocalization()
{
  QProcess::startDetached("bash", {"-c", switchModeScript() + " stop"});
  localization_active_ = false;
  navigation_active_ = false;
  updateModeButtons();
  resetDisplays();
  log("Localization stopped");
}

void LibbotControlPanel::onStopNavigation()
{
  QProcess::startDetached("bash", {"-c", switchModeScript() + " stop"});
  navigation_active_ = false;
  updateModeButtons();
  log("Navigation stopped");
}

void LibbotControlPanel::onStopAll()
{
  QProcess::startDetached("bash", {"-c", switchModeScript() + " stop"});
  mapping_active_ = false;
  localization_active_ = false;
  navigation_active_ = false;
  updateModeButtons();
  resetDisplays();
  log("Everything stopped, displays reset");
}

void LibbotControlPanel::resetDisplays()
{
  // Displays (point clouds, paths, markers) otherwise keep showing their
  // last received message forever once the node publishing them is
  // killed/replaced -- "ghosting" stale data from a previous run (e.g.
  // the localization map cloud still visible after switching straight
  // to mapping without an intermediate Stop). This panel is itself an
  // RViz plugin living in the same process, so it can trigger the same
  // reset the Displays panel's own "Reset" button does, on every mode
  // transition, not just Stop Everything.
  if (auto * ctx = getDisplayContext()) {
    if (auto * root = ctx->getRootDisplayGroup()) {
      root->reset();
    }
  }
}

void LibbotControlPanel::updateModeButtons()
{
  if (start_mapping_button_) start_mapping_button_->setEnabled(!saving_map_);
  if (stop_mapping_button_) stop_mapping_button_->setEnabled(mapping_active_);
  if (save_map_button_) save_map_button_->setEnabled(mapping_active_ && !saving_map_);

  if (start_localization_button_) start_localization_button_->setEnabled(!mapping_active_ && !saving_map_);
  if (stop_localization_button_) stop_localization_button_->setEnabled(localization_active_);

  if (start_navigation_button_) start_navigation_button_->setEnabled(localization_active_ && !mapping_active_);
  if (stop_navigation_button_) stop_navigation_button_->setEnabled(navigation_active_);
}

void LibbotControlPanel::setSavingUi(bool saving, const QString & status)
{
  saving_map_ = saving;
  if (save_map_button_) {
    save_map_button_->setText(saving ? "Saving..." : "Save Map As...");
  }
  updateModeButtons();
  if (!status.isEmpty()) {
    log(status);
  }
}

void LibbotControlPanel::onSaveMap()
{
  bool ok = false;
  QString name = QInputDialog::getText(
    this, "Save Map As", "Map name:", QLineEdit::Normal, "library", &ok);
  if (!ok) {
    return;
  }
  name = name.trimmed();
  if (name.isEmpty()) {
    return;
  }
  static const QRegularExpression valid_name("^[A-Za-z0-9_-]+$");
  if (!valid_name.match(name).hasMatch()) {
    log("Invalid name: letters/numbers/underscore/hyphen only");
    return;
  }
  QString dest = mapsDir() + "/" + name;
  if (QFile::exists(dest + ".yaml")) {
    auto reply = QMessageBox::question(
      this, "Overwrite Map?", "'" + name + "' already exists. Overwrite it?",
      QMessageBox::Yes | QMessageBox::No);
    if (reply != QMessageBox::Yes) {
      return;
    }
  }

  pending_map_name_ = name;
  setSavingUi(true, "Saving 3D map...");

  if (!map_save_client_->service_is_ready()) {
    setSavingUi(false, "Save failed: /map_save service not available (is mapping running?)");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  map_save_client_->async_send_request(
    req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto resp = future.get();
      if (!resp->success) {
        QMetaObject::invokeMethod(this, [this, resp]() {
          setSavingUi(false, "Save failed: " + QString::fromStdString(resp->message));
        });
        return;
      }
      QMetaObject::invokeMethod(this, [this]() {
        log("3D map saved, converting to 2D...");
        QString name = pending_map_name_;
        if (convert_process_) {
          convert_process_->deleteLater();
        }
        convert_process_ = new QProcess(this);
        connect(
          convert_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this, name](int exit_code, QProcess::ExitStatus) {
            if (exit_code == 0) {
              setSavingUi(false, "Map '" + name + "' saved");
              onRefreshMaps();
              int idx = map_combo_->findText(name);
              if (idx >= 0) {
                map_combo_->setCurrentIndex(idx);
              }
            } else {
              QString err = QString::fromUtf8(convert_process_->readAllStandardError());
              setSavingUi(false, "2D conversion failed: " + err);
            }
          });
        convert_process_->start(
          "ros2", {"run", "libbot_mapping", "pcd_to_2d_map.py", pcdPath(), name,
            "--output-dir", mapsDir(), "--pcd-output-dir", pcdMapsDir()});
      });
    });
}

void LibbotControlPanel::onDeleteMap()
{
  QString name = map_combo_->currentText();
  if (name.isEmpty()) {
    return;
  }
  auto reply = QMessageBox::question(
    this, "Delete Map", "Delete '" + name + "'?", QMessageBox::Yes | QMessageBox::No);
  if (reply != QMessageBox::Yes) {
    return;
  }
  QFile::remove(mapsDir() + "/" + name + ".yaml");
  QFile::remove(mapsDir() + "/" + name + ".pgm");
  QFile::remove(pcdMapsDir() + "/" + name + ".pcd");
  log("Deleted '" + name + "'");
  onRefreshMaps();
}

void LibbotControlPanel::onRefreshMaps()
{
  QString current = map_combo_->currentText();
  map_combo_->clear();
  QDir dir(mapsDir());
  QStringList yaml_files = dir.entryList({"*.yaml"}, QDir::Files, QDir::Name);
  for (const QString & f : yaml_files) {
    map_combo_->addItem(f.left(f.length() - QString(".yaml").length()));
  }
  if (map_combo_->count() == 0) {
    log("No saved maps found in " + mapsDir());
    return;
  }
  int idx = map_combo_->findText(current);
  map_combo_->setCurrentIndex(idx >= 0 ? idx : 0);
}

QString LibbotControlPanel::mapsDir() const
{
  return QDir::homePath() + "/libraryBot/ros2_ws/src/libbot_mapping/maps";
}

QString LibbotControlPanel::pcdPath() const
{
  return QDir::homePath() + "/libraryBot/ros2_ws/src/libbot_mapping/pcd_maps/latest.pcd";
}

QString LibbotControlPanel::pcdMapsDir() const
{
  return QDir::homePath() + "/libraryBot/ros2_ws/src/libbot_mapping/pcd_maps";
}

QString LibbotControlPanel::selectedMapYamlPath() const
{
  if (!map_combo_ || map_combo_->currentText().isEmpty()) {
    return QString();
  }
  return mapsDir() + "/" + map_combo_->currentText() + ".yaml";
}

void LibbotControlPanel::log(const QString & text)
{
  if (log_view_) {
    log_view_->append(text);
  }
}

}  // namespace libbot_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(libbot_rviz_plugins::LibbotControlPanel, rviz_common::Panel)
