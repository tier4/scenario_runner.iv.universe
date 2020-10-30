#include <condition_plugins/simulation_time_condition.h>

namespace condition_plugins
{

SimulationTimeCondition::SimulationTimeCondition()
  : scenario_conditions::ConditionBase { "SimulationTime" }
{}

bool SimulationTimeCondition::configure(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> simulator)
try
{
  node_ = node;

  name_ = read_optional<std::string>(node_, "Name", name_);

  duration_ =
    ros::Duration(
      read_essential<float>(node_, "Value"));

  if (not parseRule<ros::Duration>(
            read_essential<std::string>(node_, "Rule"),
            compare_))
  {
    return configured_ = false;
  }

  keep_ = read_optional<bool>(node_, "Keep", false);

  return configured_ = true;
}
catch (...)
{
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

ros::Duration SimulationTimeCondition::elapsed() const noexcept
{
  return ros::Time::now() - scenario_logger::log.begin();
}

bool SimulationTimeCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  if (keep_ && result_)
  {
    return true;
  }
  else
  {
    description_ = std::to_string(elapsed().toSec());
    return result_ = compare_(elapsed(), duration_);
  }
}

} // namespace condition_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(condition_plugins::SimulationTimeCondition, scenario_conditions::ConditionBase)

