#ifndef CONDITION_PLUGINS_SIMULATION_TIME_H_INCLUDED
#define CONDITION_PLUGINS_SIMULATION_TIME_H_INCLUDED

#include <rclcpp/rclcpp.hpp>
#include <scenario_conditions/condition_base.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_logger/logger.h>
#include <scenario_utility/scenario_utility.h>

namespace condition_plugins
{

class SimulationTimeCondition
  : public scenario_conditions::ConditionBase
{
  rclcpp::Duration duration_;

  std::string rule_;

  Comparator<rclcpp::Duration> compare_;
  rclcpp::Clock clock_;
public:
  SimulationTimeCondition();

  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> simulator) override;

  rclcpp::Duration elapsed() const noexcept;

  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
};

} // namespace condition_plugins

#endif // CONDITION_PLUGINS_SIMULATION_TIME_H_INCLUDED

