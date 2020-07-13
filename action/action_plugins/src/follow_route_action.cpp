#include "action_plugins/follow_route_action.h"
#include "scenario_utility/scenario_utility.h"
#include <ros/ros.h>

namespace action_plugins
{

FollowRouteAction::FollowRouteAction()
  : EntityActionBase {"FollowRoute"}
{}

void FollowRouteAction::configure(
  const YAML::Node& node,
  const std::vector<std::string> actors,
  const std::shared_ptr<ScenarioAPI>& api_ptr)
try
{
  node_ = node;
  actors_ = actors;
  api_ptr_ = api_ptr;

  if (actors_.empty())
  {
    SCENARIO_WARNING_ABOUT_NO_ACTORS_SPECIFIED();
  }

  call_with_essential(node_, "Params", [&](const auto& node) mutable
  {
    call_with_essential(node, "GoalPose", [&](const auto& node) mutable
    {
      const auto pose_stamped { read_essential<geometry_msgs::PoseStamped>(node, "Pose") };

      if (pose_stamped.header.frame_id != "/map")
      {
        goal_ = api_ptr_->getRelativePose(pose_stamped.header.frame_id, pose_stamped.pose);
      }
      else
      {
        goal_ = pose_stamped.pose;
      }

      shift_ = read_optional<std::string>(node, "Shift", "Center");
    });
  });
}
catch (...)
{
  SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION();
}

void FollowRouteAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  for (const auto& each : actors_)
  {
    if (not api_ptr_->sendGoalPoint(each, goal_, true, shift_))
    {
      SCENARIO_ERROR_THROW(CATEGORY(),
        type_ << "Action failed to send goal-pose to " << each << ".");
    }
  }
}

} // namespace action_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(action_plugins::FollowRouteAction, scenario_actions::EntityActionBase)
