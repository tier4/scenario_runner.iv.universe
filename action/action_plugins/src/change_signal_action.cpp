#include <action_plugins/change_signal_action.h>

namespace action_plugins
{

void ChangeSignalAction::configure(
  const YAML::Node& node,
  const std::vector<std::string> actors,
  const std::shared_ptr<ScenarioAPI>& api_ptr)
try
{
  actors_ = actors; // NOTE: ChangeSignal Action has no dependency for Actors
  api_ptr_ = api_ptr;
  node_ = node;

  call_with_essential(node_, "Params", [&](const auto& node) mutable
  {
    target_intersection_ = read_essential<std::string>(node, "TargetIntersection");
    state_ = read_essential<std::string>(node, "State");
  });
}
catch (...)
{
  SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION();
}

void ChangeSignalAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager>& intersection_manager)
{
  if (not (*intersection_manager).change(target_intersection_, state_))
  {
    SCENARIO_ERROR_THROW(CATEGORY(),
      type_ << "Action failed to change intersection state to " << state_ << ".");
  }
}

} // namespace action_plugins

PLUGINLIB_EXPORT_CLASS(action_plugins::ChangeSignalAction, scenario_actions::EntityActionBase)
