#include <action_plugins/fault_injection_action.h>

namespace action_plugins
{

void FaultInjectionAction::configure(
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
    kill_ = read_essential<std::string>(node, "Kill");
  });
}
catch (...)
{
  SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION();
}

void FaultInjectionAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  const auto command { "rosnode kill " + kill_ };

  if (::system(command.c_str()))
  {
    SCENARIO_ERROR_THROW(CATEGORY(),
      type_ << "Action failed to execute command \"" << command << "\".");
  }
}

} // namespace action_plugins

PLUGINLIB_EXPORT_CLASS(action_plugins::FaultInjectionAction, scenario_actions::EntityActionBase)
