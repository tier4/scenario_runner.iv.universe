#ifndef TEST_CONDITIONS_ALWAYS_TRUE_CONDITION_H_INCLUDED
#define TEST_CONDITIONS_ALWAYS_TRUE_CONDITION_H_INCLUDED

#include <scenario_conditions/condition_base.hpp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_utility/scenario_utility.hpp>

namespace condition_plugins
{

class AlwaysTrueCondition
  : public scenario_conditions::ConditionBase
{
public:
  AlwaysTrueCondition();

  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;

  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) override;
};

} // namespace condition_plugins

#endif // TEST_CONDITIONS_ALWAYS_TRUE_CONDITION_H_INCLUDED

