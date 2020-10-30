#ifndef INCLUDED_CONDITION_PLUGINS_RELATIVE_DISTANCE_CONDITION_H
#define INCLUDED_CONDITION_PLUGINS_RELATIVE_DISTANCE_CONDITION_H

#include <scenario_conditions/condition_base.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_logger/logger.h>
#include <scenario_utility/scenario_utility.h>

namespace condition_plugins
{

class RelativeDistanceCondition
  : public scenario_conditions::ConditionBase
{
  std::string trigger_, target_entity_, rule_;

  float target_value_;

  Comparator<float> compare_;

public:
  RelativeDistanceCondition();

  bool configure(YAML::Node, std::shared_ptr<ScenarioAPI>) override;

  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
};

} // namespace condition_plugins


#endif // INCLUDED_CONDITION_PLUGINS_RELATIVE_DISTANCE_CONDITION_H

