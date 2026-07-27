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
// Gazebo Classic flattens that <include> into a single physics::Model:
// GetLinks() on the outer "BookE_01_001" model returns the included
// model's link directly, but its GetName() keeps the included model's
// name as a prefix (e.g. "aws_robomaker_retail_BookE_01::link") rather
// than a bare "link" -- there is no separate nested Model object to
// recurse into. Match on the part of the name after the last "::" so
// callers can just name the leaf link ("link", "gripper_link", ...)
// regardless of how deep an included model's own internal scoping goes.
std::string LeafName(const std::string & scoped_name)
{
  auto pos = scoped_name.rfind("::");
  return pos == std::string::npos ? scoped_name : scoped_name.substr(pos + 2);
}

gazebo::physics::LinkPtr FindLinkRecursive(
  const gazebo::physics::ModelPtr & model, const std::string & link_name)
{
  for (const auto & link : model->GetLinks()) {
    if (LeafName(link->GetName()) == link_name) {
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

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&LinkAttacherPlugin::OnWorldUpdate, this));

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "libbot_link_attacher ready (~/attach_link, ~/detach_link)");
}

void LinkAttacherPlugin::OnWorldUpdate()
{
  std::lock_guard<std::mutex> lock(joints_mutex_);
  for (auto & [key, attachment] : joints_) {
    (void)key;
    attachment.link2->SetWorldPose(attachment.relative_pose + attachment.link1->WorldPose());
  }
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
    // Already attached -- treat as success, no duplicate attachment.
    response->success = true;
    return;
  }

  // A dynamically created ODE "fixed" joint between two independent
  // top-level models (this robot's gripper_link and a book model) turned
  // out to resonate into a runaway tumble within a few physics steps no
  // matter how the joint/mass/velocity were set up -- ODE's constraint
  // solver just isn't a good fit for welding two unrelated models at
  // runtime here. Instead, keep the book kinematically following the
  // gripper's pose every world update (see OnWorldUpdate) -- no
  // constraint solver involved, so nothing to destabilize.
  //
  // Book props are spawned static (see library.world) so they don't
  // drift on their own. Keep it static for the follow: SetWorldPose on a
  // static link is a plain teleport with no dynamics to fight, whereas a
  // dynamic body would still need the physics engine to reconcile
  // gravity/contacts against our every-step pose override.
  const ignition::math::Pose3d relative_pose = link2->WorldPose() - link1->WorldPose();
  joints_[key] = Attachment{link1, link2, relative_pose};
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

  joints_.erase(it);
  response->success = true;
}

GZ_REGISTER_WORLD_PLUGIN(LinkAttacherPlugin)

}  // namespace libbot_gazebo_plugins
