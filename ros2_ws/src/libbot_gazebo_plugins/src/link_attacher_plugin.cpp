#include "libbot_gazebo_plugins/link_attacher_plugin.hpp"

#include <gazebo/common/Console.hh>

namespace libbot_gazebo_plugins
{

std::string LinkAttacherPlugin::JointKey(
  const std::string & model1, const std::string & link1,
  const std::string & model2, const std::string & link2)
{
  return model1 + "::" + link1 + "|" + model2 + "::" + link2;
}

namespace
{
// Props in library.world are spawned as e.g.
// <model name="BookE_01_001"><include><uri>model://aws_robomaker_retail_BookE_01</uri>...
// which nests the included model's own link one level down
// (BookE_01_001::aws_robomaker_retail_BookE_01::link), not directly on
// the outer model. Model::GetLink() only checks that model's own direct
// links, so callers would otherwise need to know each prop's inner model
// name. Recursing into NestedModels() lets attach/detach requests just
// name the leaf link ("link", "gripper_link", ...) regardless of nesting.
gazebo::physics::LinkPtr FindLinkRecursive(
  const gazebo::physics::ModelPtr & model, const std::string & link_name)
{
  for (const auto & link : model->GetLinks()) {
    if (link->GetName() == link_name) {
      return link;
    }
  }
  for (const auto & nested : model->NestedModels()) {
    auto found = FindLinkRecursive(nested, link_name);
    if (found) {
      return found;
    }
  }
  return nullptr;
}
}  // namespace

void LinkAttacherPlugin::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
  world_ = world;
  ros_node_ = gazebo_ros::Node::Get(sdf);

  attach_service_ = ros_node_->create_service<srv::AttachLink>(
    "attach_link",
    std::bind(
      &LinkAttacherPlugin::OnAttach, this,
      std::placeholders::_1, std::placeholders::_2));

  detach_service_ = ros_node_->create_service<srv::DetachLink>(
    "detach_link",
    std::bind(
      &LinkAttacherPlugin::OnDetach, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "libbot_link_attacher ready (~/attach_link, ~/detach_link)");
}

void LinkAttacherPlugin::OnAttach(
  const std::shared_ptr<srv::AttachLink::Request> request,
  std::shared_ptr<srv::AttachLink::Response> response)
{
  auto model1 = world_->ModelByName(request->model1_name);
  auto model2 = world_->ModelByName(request->model2_name);
  if (!model1 || !model2) {
    RCLCPP_WARN(
      ros_node_->get_logger(), "attach_link: model '%s' or '%s' not found",
      request->model1_name.c_str(), request->model2_name.c_str());
    response->success = false;
    return;
  }

  auto link1 = FindLinkRecursive(model1, request->link1_name);
  auto link2 = FindLinkRecursive(model2, request->link2_name);
  if (!link1 || !link2) {
    RCLCPP_WARN(
      ros_node_->get_logger(), "attach_link: link '%s' or '%s' not found",
      request->link1_name.c_str(), request->link2_name.c_str());
    response->success = false;
    return;
  }

  const std::string key = JointKey(
    request->model1_name, request->link1_name,
    request->model2_name, request->link2_name);

  std::lock_guard<std::mutex> lock(joints_mutex_);
  if (joints_.count(key)) {
    // Already attached -- treat as success, no duplicate joint.
    response->success = true;
    return;
  }

  // Book props are spawned static (mass 0 -- they're decorative furniture
  // props, see library.world) so they don't drift on their own, but a
  // static body also doesn't respond to joint forces in ODE: a fixed
  // joint to a still-static book would just anchor the gripper in place
  // rather than the book actually following it. Make the book dynamic for
  // the duration of the carry; OnDetach sets it back to static once
  // released so it rests exactly where placed like every other prop.
  model2->SetStatic(false);

  gazebo::physics::JointPtr joint = model1->CreateJoint(
    "libbot_attach_" + key, "fixed", link1, link2);
  if (!joint) {
    model2->SetStatic(true);
    response->success = false;
    return;
  }
  joint->Init();
  joints_[key] = joint;
  response->success = true;
}

void LinkAttacherPlugin::OnDetach(
  const std::shared_ptr<srv::DetachLink::Request> request,
  std::shared_ptr<srv::DetachLink::Response> response)
{
  const std::string key = JointKey(
    request->model1_name, request->link1_name,
    request->model2_name, request->link2_name);

  std::lock_guard<std::mutex> lock(joints_mutex_);
  auto it = joints_.find(key);
  if (it == joints_.end()) {
    response->success = false;
    return;
  }

  it->second->Detach();
  it->second->Fini();
  joints_.erase(it);

  auto model2 = world_->ModelByName(request->model2_name);
  if (model2) {
    model2->SetStatic(true);
  }
  response->success = true;
}

GZ_REGISTER_WORLD_PLUGIN(LinkAttacherPlugin)

}  // namespace libbot_gazebo_plugins
