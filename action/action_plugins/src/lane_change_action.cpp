#include <action_plugins/lane_change_action.h>

namespace action_plugins
{

LaneChangeAction::LaneChangeAction()
  : EntityActionBase {"LaneChange"}
{}

void LaneChangeAction::configure(
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

  name_ = read_optional<std::string>(node_, "Name", name_);

  call_with_essential(node_, "Params", [&](const auto& node) mutable
  {
    target_lanelet_ = read_essential<int>(node, "TargetLanelet");
  });
}
catch (...)
{
  SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION();
}

void LaneChangeAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  for (const auto& each : actors_)
  {
    if (not (*api_ptr_).changeNPCLaneChange(each, target_lanelet_))
    {
      SCENARIO_ERROR_THROW(CATEGORY(),
        type_ << "Action failed to change lane of " << each << ".");
    }
  }
}

} // namespace action_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(action_plugins::LaneChangeAction, scenario_actions::EntityActionBase)
