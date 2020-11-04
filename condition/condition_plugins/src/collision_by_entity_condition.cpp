#include <condition_plugins/collision_by_entity_condition.h>
#include <scenario_logger/logger.h>

namespace condition_plugins
{

CollisionByEntityCondition::CollisionByEntityCondition()
  : scenario_conditions::ConditionBase { "CollisionByEntity" }
{}

bool CollisionByEntityCondition::configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr)
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

  keep_ = read_optional<bool>(node_, "Keep", false);

  return configured_ = true;
}
catch (...)
{
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

bool CollisionByEntityCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  if (!configured_)
  {
    SCENARIO_THROW_ERROR_ABOUT_INCOMPLETE_CONFIGURATION();
  }

  if (keep_ && result_)
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

    return result_ = (distance <= std::numeric_limits<double>::epsilon());
  }
}

}  // namespace condition_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(condition_plugins::CollisionByEntityCondition, scenario_conditions::ConditionBase)

