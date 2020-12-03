#ifndef INCLUDED_CONDITION_PLUGINS_SIGNAL_CONDITION_H
#define INCLUDED_CONDITION_PLUGINS_SIGNAL_CONDITION_H

#include <scenario_conditions/condition_base.hpp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_logger/logger.hpp>
#include <scenario_utility/scenario_utility.hpp>

namespace condition_plugins
{

class SignalCondition
  : public scenario_conditions::ConditionBase
{
  std::shared_ptr<ScenarioAPI> simulator_;

  std::string trigger_, state_;

public:
  SignalCondition();

  bool configure(YAML::Node, std::shared_ptr<ScenarioAPI>) override;

  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
};

} // namespace condition_plugins

#endif // INCLUDED_CONDITION_PLUGINS_SIGNAL_CONDITION_H

