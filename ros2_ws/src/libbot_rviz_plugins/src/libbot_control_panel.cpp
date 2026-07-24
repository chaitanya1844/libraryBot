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

  auto * step1 = new QPushButton("1. Start Mapping");
  auto * step1b = new QPushButton("Save Map");
  auto * step2 = new QPushButton("2. Start Localization");
  auto * step3 = new QPushButton("3. Start Navigation");
  connect(step1, &QPushButton::clicked, this, &LibbotControlPanel::onStartMapping);
  connect(step1b, &QPushButton::clicked, this, &LibbotControlPanel::onSaveMap);
  connect(step2, &QPushButton::clicked, this, &LibbotControlPanel::onStartLocalization);
  connect(step3, &QPushButton::clicked, this, &LibbotControlPanel::onStartNavigation);

  mode_grid->addWidget(step1, 0, 0);
  mode_grid->addWidget(step1b, 0, 1);
  mode_grid->addWidget(step2, 1, 0, 1, 2);
  mode_grid->addWidget(step3, 2, 0, 1, 2);

  auto * mode_help = new QLabel(
    "<small>1. Start Mapping, then drive around with the Xbox controller "
    "(<code>ros2 launch libbot_control teleop_joy.launch.py</code>, hold LB) "
    "until the map looks complete in RViz, then Save Map.<br>"
    "2. Start Localization to load the saved map and locate the robot on "
    "it (AMCL) -- check the particle cloud converges in RViz.<br>"
    "3. Start Navigation to bring up Nav2. Then use RViz's own "
    "\"2D Pose Estimate\" tool to set where the robot starts and "
    "\"Nav2 Goal\" to send it anywhere on the map.</small>");
  mode_help->setWordWrap(true);
  mode_grid->addWidget(mode_help, 3, 0, 1, 2);

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

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &LibbotControlPanel::onSpinTimer);
  spin_timer_->start(50);

  ui_timer_ = new QTimer(this);
  connect(ui_timer_, &QTimer::timeout, this, &LibbotControlPanel::onUiTimer);
  ui_timer_->start(300);

  log("Panel initialized");
}

void LibbotControlPanel::onSpinTimer()
{
  if (node_) {
    rclcpp::spin_some(node_);
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
  QString map_path = QDir::homePath() + "/libraryBot/ros2_ws/src/libbot_mapping/maps/library.yaml";
  QProcess::startDetached("bash", {"-c", switchModeScript() + " localization " + map_path});
  log("Stopping mapping, starting localization (AMCL) against the saved map...");
}

void LibbotControlPanel::onStartNavigation()
{
  QProcess::startDetached("bash", {"-c", switchModeScript() + " navigation"});
  log("Starting Nav2 (requires localization already running)");
}

void LibbotControlPanel::onSaveMap()
{
  QProcess::startDetached(
    "ros2", {"run", "nav2_map_server", "map_saver_cli", "-f",
      QDir::homePath() + "/libraryBot/ros2_ws/src/libbot_mapping/maps/library"});
  log("Saving map...");
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
