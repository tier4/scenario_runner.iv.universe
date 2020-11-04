#include <condition_plugins/speed_condition.h>

namespace condition_plugins
{

SpeedCondition::SpeedCondition()
  : scenario_conditions::ConditionBase { "Speed" }
{}

bool SpeedCondition::configure(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> api_ptr)
try
{
  node_ = node;
  api_ptr_ = api_ptr;

  name_ = read_optional<std::string>(node_, "Name", name_);

  trigger_ = read_essential<std::string>(node_, "Trigger");

  target_value_ = read_essential<float>(node_, "Value");

  if (not parseRule<float>(read_essential<std::string>(node_, "Rule"), compare_))
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

bool SpeedCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  if (keep_ && result_)
  {
    return true;
  }
  else
  {
    if ((*api_ptr_).isEgoCarName(trigger_))
    {
      description_ = std::to_string((*api_ptr_).getVelocity());
      return result_ = compare_((*api_ptr_).getVelocity(), target_value_);
    }
    else
    {
      double velocity { 0 };

      if (not (*api_ptr_).getNPCVelocity(trigger_, &velocity))
      {
        ROS_ERROR_STREAM("Invalid trigger name specified for " << getType() << " condition named " << getName());
        return result_ = false;
      }
      else
      {
        description_ = std::to_string(velocity);
        return result_ = compare_(velocity, target_value_);
      }
    }
  }
}

}  // namespace condition_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(condition_plugins::SpeedCondition, scenario_conditions::ConditionBase)

