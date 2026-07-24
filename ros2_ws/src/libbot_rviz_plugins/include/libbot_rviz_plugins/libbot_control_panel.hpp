#ifndef LIBBOT_RVIZ_PLUGINS__LIBBOT_CONTROL_PANEL_HPP_
#define LIBBOT_RVIZ_PLUGINS__LIBBOT_CONTROL_PANEL_HPP_

#include <QtWidgets>
#undef NO_ERROR

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

namespace libbot_rviz_plugins
{

/// RViz-embedded control panel for libraryBot: e-stop, camera view, and
/// mode switching (map/localize/navigate), all inside the same window as
/// the 3D view (no separate app to focus/lose track of). Manual driving
/// is the Xbox controller (libbot_control/teleop_joy.launch.py); giving
/// Nav2 a goal is RViz's own "2D Pose Estimate" / "Nav2 Goal" toolbar
/// tools, both already in libbot_nav.rviz.
class LibbotControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit LibbotControlPanel(QWidget * parent = nullptr);
  ~LibbotControlPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onEStopToggled(bool checked);
  void onStartMapping();
  void onStartLocalization();
  void onStartNavigation();
  void onSaveMap();
  void onOpenCameraView();
  void onSpinTimer();
  void onUiTimer();

private:
  void setEStop(bool engaged);
  void log(const QString & text);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::mutex state_mutex_;

  bool estop_active_{false};

  QTimer * spin_timer_{nullptr};
  QTimer * ui_timer_{nullptr};

  // status
  QLabel * pose_label_{nullptr};

  // manual drive
  QPushButton * estop_button_{nullptr};

  QTextEdit * log_view_{nullptr};
};

}  // namespace libbot_rviz_plugins

#endif  // LIBBOT_RVIZ_PLUGINS__LIBBOT_CONTROL_PANEL_HPP_
