#ifndef CONDITION_PLUGINS_SPEED_CONDITION_H_INCLUDED
#define CONDITION_PLUGINS_SPEED_CONDITION_H_INCLUDED

#include <scenario_conditions/condition_base.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_logger/logger.h>
#include <scenario_utility/scenario_utility.h>
#include <sstream>

namespace condition_plugins
{

class SpeedCondition : public scenario_conditions::ConditionBase
{
public:
  SpeedCondition();
  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) override;

private:
  std::string rule_;
  std::string trigger_;
  float target_value_;
  Comparator<float> compare_;
};

}  // namespace condition_plugins

#endif  // CONDITION_PLUGINS_SPEED_CONDITION_H_INCLUDED
