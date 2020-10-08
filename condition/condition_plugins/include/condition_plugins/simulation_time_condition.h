#ifndef CONDITION_PLUGINS_SIMULATION_TIME_H_INCLUDED
#define CONDITION_PLUGINS_SIMULATION_TIME_H_INCLUDED

#include <scenario_conditions/condition_base.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_logger/logger.h>
#include <scenario_utility/scenario_utility.h>

namespace condition_plugins
{

class SimulationTimeCondition
  : public scenario_conditions::ConditionBase
{
  ros::Duration duration_;

  std::string rule_;

  Comparator<ros::Duration> compare_;

public:
  SimulationTimeCondition();

  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> simulator) override;

  ros::Duration elapsed() const noexcept;

  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
};

} // namespace condition_plugins

#endif // CONDITION_PLUGINS_SIMULATION_TIME_H_INCLUDED

