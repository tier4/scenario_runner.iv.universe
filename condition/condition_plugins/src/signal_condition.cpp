#include <condition_plugins/signal_condition.h>

namespace condition_plugins
{

SignalCondition::SignalCondition()
  : scenario_conditions::ConditionBase { "Signal" }
{}

bool SignalCondition::configure(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> simulator)
try
{
  node_ = node;

  simulator_ = simulator;

  name_ = read_optional<std::string>(node_, "Name", name_);

  trigger_ = read_essential<std::string>(node_, "Trigger");

  state_ = read_essential<std::string>(node_, "State");

  keep_ = read_optional<bool>(node_, "Keep", false);

  return configured_ = true;
}
catch (...)
{
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

bool SignalCondition::update(const std::shared_ptr<scenario_intersection::IntersectionManager> & intersections)
{
  // ROS_WARN_STREAM("IDs = [");
  //
  // for (const auto& id : (*intersections).at(trigger_).ids())
  // {
  //   ROS_WARN_STREAM("  " << id);
  // }
  //
  // ROS_WARN_STREAM("]");

  if (keep_ && result_)
  {
    return result_;
  }
  else
  {
    description_ = (*intersections).at(trigger_).current_state();
    return result_ = (*intersections).at(trigger_).is(state_);
  }
}

} // namespace condition_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(condition_plugins::SignalCondition, scenario_conditions::ConditionBase)
