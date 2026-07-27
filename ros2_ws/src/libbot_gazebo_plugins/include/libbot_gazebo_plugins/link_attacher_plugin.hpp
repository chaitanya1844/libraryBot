#ifndef LIBBOT_GAZEBO_PLUGINS__LINK_ATTACHER_PLUGIN_HPP_
#define LIBBOT_GAZEBO_PLUGINS__LINK_ATTACHER_PLUGIN_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include "libbot_gazebo_plugins/srv/attach_link.hpp"
#include "libbot_gazebo_plugins/srv/detach_link.hpp"

namespace libbot_gazebo_plugins
{

// World plugin providing /libbot/attach_link and /libbot/detach_link
// services. Gazebo Classic has no built-in grasp/suction concept, so the
// arm-based book pick/place task manager (libbot_manipulation) uses this
// to rigidly weld a book model's link to the gripper link on grasp, and
// break that joint again on place, rather than relying on real contact
// friction to hold a book in the gripper.
class LinkAttacherPlugin : public gazebo::WorldPlugin
{
public:
  LinkAttacherPlugin() = default;
  ~LinkAttacherPlugin() override = default;

  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
  static std::string JointKey(
    const std::string & model1, const std::string & link1,
    const std::string & model2, const std::string & link2);

  void OnAttach(
    const std::shared_ptr<srv::AttachLink::Request> request,
    std::shared_ptr<srv::AttachLink::Response> response);

  void OnDetach(
    const std::shared_ptr<srv::DetachLink::Request> request,
    std::shared_ptr<srv::DetachLink::Response> response);

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Service<srv::AttachLink>::SharedPtr attach_service_;
  rclcpp::Service<srv::DetachLink>::SharedPtr detach_service_;

  std::mutex joints_mutex_;
  std::map<std::string, gazebo::physics::JointPtr> joints_;
};

}  // namespace libbot_gazebo_plugins

#endif  // LIBBOT_GAZEBO_PLUGINS__LINK_ATTACHER_PLUGIN_HPP_
