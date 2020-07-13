#include <action_plugins/enable_action.h>

namespace action_plugins
{

EnableAction::EnableAction()
  : EntityActionBase { "Enable" }
{}

void EnableAction::configure(
  const YAML::Node& node,
  const std::vector<std::string> actors,
  const std::shared_ptr<ScenarioAPI>& simulator)
try
{
  node_ = node;
  actors_ = actors;
  api_ptr_ = simulator;

  if (actors_.empty())
  {
    SCENARIO_WARNING_ABOUT_NO_ACTORS_SPECIFIED();
  }

  name_ = read_optional<std::string>(node_, "Name", name_);

  call_with_essential(node_, "Params", [&](const auto& node) mutable
  {
    automatic_emergency_braking_ =
      read_optional<bool>(node, "AutomaticEmergencyBraking");
  });
}
catch (...)
{
  SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION();
}

void EnableAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  if (automatic_emergency_braking_)
  {
    for (const auto& each : actors_)
    {
      if (not (*api_ptr_).changeNPCConsiderVehicle(each, *automatic_emergency_braking_))
      {
        SCENARIO_ERROR_THROW(CATEGORY(),
          type_ << "Action failed to change " << each << "'s feature.");
      }
    }
  }
}

}

PLUGINLIB_EXPORT_CLASS(action_plugins::EnableAction, scenario_actions::EntityActionBase)
