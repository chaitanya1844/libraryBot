#ifndef LIBBOT_RVIZ_PLUGINS__LIBBOT_CONTROL_PANEL_HPP_
#define LIBBOT_RVIZ_PLUGINS__LIBBOT_CONTROL_PANEL_HPP_

#include <QtWidgets>
#include <QRegularExpression>
#undef NO_ERROR

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

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
  void onTeleopToggled(bool checked);
  void onTeleopForward();
  void onTeleopBack();
  void onTeleopStrafeLeft();
  void onTeleopStrafeRight();
  void onTeleopRotateCw();
  void onTeleopRotateCcw();
  void onTeleopStop();
  void onTeleopReleased();
  void onStartMapping();
  void onStartLocalization();
  void onStartNavigation();
  void onSaveMap();
  void onOpenCameraView();
  void onRefreshMaps();
  void onSpinTimer();
  void onUiTimer();

private:
  void setEStop(bool engaged);
  void setTeleopButtonsEnabled(bool enabled);
  void log(const QString & text);
  QString mapsDir() const;
  QString selectedMapYamlPath() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::mutex state_mutex_;

  bool estop_active_{false};
  bool teleop_active_{false};
  bool teleop_key_held_{false};
  geometry_msgs::msg::Twist teleop_cmd_;

  QTimer * spin_timer_{nullptr};
  QTimer * ui_timer_{nullptr};

  // status
  QLabel * pose_label_{nullptr};

  // map selection (localization/navigation) + save-as name
  QComboBox * map_combo_{nullptr};

  // manual drive
  QPushButton * estop_button_{nullptr};
  QPushButton * teleop_button_{nullptr};
  QPushButton * teleop_forward_{nullptr};
  QPushButton * teleop_back_{nullptr};
  QPushButton * teleop_strafe_left_{nullptr};
  QPushButton * teleop_strafe_right_{nullptr};
  QPushButton * teleop_rotate_cw_{nullptr};
  QPushButton * teleop_rotate_ccw_{nullptr};
  QPushButton * teleop_stop_{nullptr};

  QTextEdit * log_view_{nullptr};
};

}  // namespace libbot_rviz_plugins

#endif  // LIBBOT_RVIZ_PLUGINS__LIBBOT_CONTROL_PANEL_HPP_
