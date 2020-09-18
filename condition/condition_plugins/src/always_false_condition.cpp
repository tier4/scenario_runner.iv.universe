#include <condition_plugins/always_false_condition.h>

namespace test_conditions
{

AlwaysFalseCondition::AlwaysFalseCondition()
  : scenario_conditions::ConditionBase {"AlwaysFalse"}
{}

bool AlwaysFalseCondition::configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr)
try
{
  node_    = node;
  api_ptr_ = api_ptr;
  name_    = read_optional<std::string>(node_, "Name", name_);

  return configured_ = true;
}
catch (...)
{
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

bool AlwaysFalseCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  return false;
}

} // namespace test_conditions

PLUGINLIB_EXPORT_CLASS(test_conditions::AlwaysFalseCondition, scenario_conditions::ConditionBase)

