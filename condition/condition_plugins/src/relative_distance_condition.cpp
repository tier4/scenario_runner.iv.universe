#include <condition_plugins/relative_distance_condition.h>

namespace condition_plugins
{

RelativeDistanceCondition::RelativeDistanceCondition()
  : scenario_conditions::ConditionBase { "RelativeDistance" }
{}

bool RelativeDistanceCondition::configure(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> api_ptr)
try
{
  node_ = node;

  api_ptr_ = api_ptr;

  name_ = read_optional<std::string>(node_, "Name", name_);

  trigger_ = read_essential<std::string>(node_, "Trigger");

  target_entity_ = read_essential<std::string>(node_, "TargetEntity");

  if ((*api_ptr_).isEgoCarName(target_entity_))
  {
    std::swap(trigger_, target_entity_);
  }

  target_value_ = read_essential<float>(node_, "Value");

  if (not parseRule<float>(read_essential<std::string>(node_, "Rule"), compare_))
  {
    return configured_ = false;
  }

  keep_ = read_optional<bool>(node_, "Keep", false);

  return configured_= true;
}
catch (...)
{
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

bool RelativeDistanceCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  if (keep_ and result_)
  {
    return result_;
  }
  else
  {
    double distance {};

    if ((*api_ptr_).isEgoCarName(trigger_))
    {
      (*api_ptr_).calcDistToNPC(distance, target_entity_);
    }
    else
    {
      (*api_ptr_).calcDistToNPCFromNPC(distance, trigger_, target_entity_);
    }

    description_ = std::to_string(distance);

    return result_ = compare_(distance, target_value_);
  }
}

} // namespace condition_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(condition_plugins::RelativeDistanceCondition, scenario_conditions::ConditionBase)

