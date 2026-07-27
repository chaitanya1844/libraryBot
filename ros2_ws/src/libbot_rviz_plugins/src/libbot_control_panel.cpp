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

  // ---------------- camera ----------------
  // RViz's own Image display crashes this GPU/driver combination
  // (separate GL render target -> "libGL error: failed to create
  // drawable" -> Ogre out-of-memory -> SIGABRT), so the D435i color feed
  // is shown via rqt_image_view instead, which renders in its own
  // process/GL context and doesn't share rviz2's.
  auto * camera_box = new QGroupBox("Camera");
  auto * camera_v = new QVBoxLayout(camera_box);
  auto * camera_btn = new QPushButton("Open Camera View");
  connect(camera_btn, &QPushButton::clicked, this, &LibbotControlPanel::onOpenCameraView);
  camera_v->addWidget(camera_btn);
  root->addWidget(camera_box);

  // ---------------- mode: Map -> Localize -> Navigate ----------------
  auto * mode_box = new QGroupBox("Mode (do these in order, each stops the previous one)");
  auto * mode_grid = new QGridLayout(mode_box);

  auto * map_label = new QLabel("Map:");
  map_combo_ = new QComboBox();
  map_combo_->setToolTip(
    "Which saved map Start Localization/Navigation use. Refreshed automatically "
    "after Save Map, or click Refresh Maps after adding one outside this panel.");
  auto * refresh_maps_btn = new QPushButton("Refresh Maps");
  connect(refresh_maps_btn, &QPushButton::clicked, this, &LibbotControlPanel::onRefreshMaps);
  mode_grid->addWidget(map_label, 0, 0);
  mode_grid->addWidget(map_combo_, 0, 1);
  mode_grid->addWidget(refresh_maps_btn, 0, 2);

  auto * step1 = new QPushButton("1. Start Mapping");
  auto * step1b = new QPushButton("Save Map As...");
  auto * step2 = new QPushButton("2. Start Localization");
  auto * step3 = new QPushButton("3. Start Navigation");
  connect(step1, &QPushButton::clicked, this, &LibbotControlPanel::onStartMapping);
  connect(step1b, &QPushButton::clicked, this, &LibbotControlPanel::onSaveMap);
  connect(step2, &QPushButton::clicked, this, &LibbotControlPanel::onStartLocalization);
  connect(step3, &QPushButton::clicked, this, &LibbotControlPanel::onStartNavigation);

  mode_grid->addWidget(step1, 1, 0);
  mode_grid->addWidget(step1b, 1, 1, 1, 2);
  mode_grid->addWidget(step2, 2, 0, 1, 3);
  mode_grid->addWidget(step3, 3, 0, 1, 3);

  auto * mode_help = new QLabel(
    "<small>1. Start Mapping, then drive around with the Xbox controller "
    "(<code>ros2 launch libbot_control teleop_joy.launch.py</code>, hold LB) "
    "until the map looks complete in RViz, then Save Map As... (pick a name).<br>"
    "2. Pick a map above, then Start Localization to load it and locate the "
    "robot on it via FAST-LIO2 (3D lidar-inertial odometry) -- enable the "
    "FastLioCloud/FastLioPath displays in RViz to check it's tracking.<br>"
    "3. Start Navigation to bring up Nav2. Then use RViz's own "
    "\"2D Pose Estimate\" tool to set where the robot starts and "
    "\"Nav2 Goal\" to send it anywhere on the map.</small>");
  mode_help->setWordWrap(true);
  mode_grid->addWidget(mode_help, 4, 0, 1, 3);

  root->addWidget(mode_box);

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
    log(engaged
      ? "E-stop engaged (controller_manager not reachable -- zero Twist only)"
      : "E-stop released (controller_manager not reachable)");
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
  log(engaged ? "E-stop engaged: drive controller deactivated"
              : "E-stop released: drive controller reactivated");
}

void LibbotControlPanel::onTeleopToggled(bool checked)
{
  teleop_active_ = checked;
  teleop_button_->setText(checked ? "Teleop Override ENGAGED (click to release)" : "Teleop Override");
  setTeleopButtonsEnabled(checked);

  if (checked) {
    if (nav_action_client_ && nav_action_client_->action_server_is_ready()) {
      nav_action_client_->async_cancel_all_goals();
      log("Teleop override engaged: canceled any active Nav2 goal");
    } else {
      log("Teleop override engaged (Nav2 action server not reachable -- nothing to cancel)");
    }
  } else {
    teleop_key_held_ = false;
    teleop_cmd_ = geometry_msgs::msg::Twist();
    cmd_pub_->publish(teleop_cmd_);
    log("Teleop override released -- send a new Nav2 goal to resume autonomous driving");
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

void LibbotControlPanel::onOpenCameraView()
{
  QProcess::startDetached(
    "bash", {"-c", "ros2 run rqt_image_view rqt_image_view /libbot/camera/camera/image_raw"});
  log("Opening camera view (rqt_image_view) in a separate window...");
}

void LibbotControlPanel::onStartMapping()
{
  QProcess::startDetached("bash", {"-c", switchModeScript() + " mapping"});
  log("Stopping any localization/navigation, starting mapping (SLAM)...");
}

void LibbotControlPanel::onStartLocalization()
{
  QString map_path = selectedMapYamlPath();
  if (map_path.isEmpty()) {
    log("No map selected -- pick one from the Map dropdown (or Save Map As... first)");
    return;
  }
  QProcess::startDetached("bash", {"-c", switchModeScript() + " localization \"" + map_path + "\""});
  log("Stopping mapping, starting localization (FAST-LIO2) against '" +
      map_combo_->currentText() + "'...");
}

void LibbotControlPanel::onStartNavigation()
{
  QProcess::startDetached("bash", {"-c", switchModeScript() + " navigation"});
  log("Starting Nav2 (requires localization already running)");
}

void LibbotControlPanel::onSaveMap()
{
  bool ok = false;
  QString name = QInputDialog::getText(
    this, "Save Map As", "Map name (no extension):", QLineEdit::Normal, "library", &ok);
  if (!ok) {
    return;
  }
  name = name.trimmed();
  if (name.isEmpty()) {
    log("Save Map cancelled: name was empty");
    return;
  }
  static const QRegularExpression valid_name("^[A-Za-z0-9_-]+$");
  if (!valid_name.match(name).hasMatch()) {
    log("Save Map cancelled: name must be letters/numbers/underscore/hyphen only "
        "(no spaces or slashes) -- got '" + name + "'");
    return;
  }
  QString dest = mapsDir() + "/" + name;
  if (QFile::exists(dest + ".yaml")) {
    auto reply = QMessageBox::question(
      this, "Overwrite Map?", "'" + name + "' already exists. Overwrite it?",
      QMessageBox::Yes | QMessageBox::No);
    if (reply != QMessageBox::Yes) {
      log("Save Map cancelled: '" + name + "' already exists");
      return;
    }
  }
  QProcess::startDetached("ros2", {"run", "nav2_map_server", "map_saver_cli", "-f", dest});
  log("Saving map as '" + name + "'...");
  // map_saver_cli runs detached (fire-and-forget, matches the rest of this
  // panel's process launches); give it a moment to actually write the
  // files before refreshing the dropdown, then select what was just saved.
  QTimer::singleShot(1500, this, [this, name]() {
    onRefreshMaps();
    int idx = map_combo_->findText(name);
    if (idx >= 0) {
      map_combo_->setCurrentIndex(idx);
    }
  });
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
