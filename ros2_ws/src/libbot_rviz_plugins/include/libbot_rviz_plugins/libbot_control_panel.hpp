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
#include "libbot_msgs/msg/sort_task_status.hpp"
#include "libbot_msgs/srv/sort_book.hpp"

namespace libbot_rviz_plugins
{

/// RViz-embedded control panel for libraryBot: manual drive, e-stop,
/// sorting controls, and mode switching, all inside the same window as
/// the 3D view (no separate app to focus/lose track of).
class LibbotControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit LibbotControlPanel(QWidget * parent = nullptr);
  ~LibbotControlPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onEStopToggled(bool checked);
  void onStartSorting();
  void onPauseSorting();
  void onQueueManualSort();
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
  rclcpp::Subscription<libbot_msgs::msg::SortTaskStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_enabled_client_;
  rclcpp::Client<libbot_msgs::srv::SortBook>::SharedPtr sort_book_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

  libbot_msgs::msg::SortTaskStatus::SharedPtr latest_status_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::mutex state_mutex_;

  bool estop_active_{false};

  QTimer * spin_timer_{nullptr};
  QTimer * ui_timer_{nullptr};

  // status
  QLabel * state_label_{nullptr};
  QLabel * tag_label_{nullptr};
  QLabel * shelf_label_{nullptr};
  QLabel * message_label_{nullptr};
  QLabel * count_label_{nullptr};
  QLabel * pose_label_{nullptr};

  // manual drive
  QPushButton * estop_button_{nullptr};

  // sorting
  QLineEdit * tag_input_{nullptr};
  QLineEdit * title_input_{nullptr};
  QComboBox * genre_combo_{nullptr};

  QTextEdit * log_view_{nullptr};
};

}  // namespace libbot_rviz_plugins

#endif  // LIBBOT_RVIZ_PLUGINS__LIBBOT_CONTROL_PANEL_HPP_
