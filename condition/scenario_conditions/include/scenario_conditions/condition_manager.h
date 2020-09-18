#ifndef SCENARIO_CONDITION_CONDITION_MANAGER_H_INCLUDED
#define SCENARIO_CONDITION_CONDITION_MANAGER_H_INCLUDED

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <scenario_conditions/condition_base.h>
#include <scenario_conditions/condition_visualizer.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_conditions
{
  class ConditionManager
  {
  public:
    ConditionManager(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr);

    simulation_is update(
      const std::shared_ptr<scenario_intersection::IntersectionManager>&);

    using condition_type
      = boost::shared_ptr<scenario_conditions::ConditionBase>;

    void applyVisitorForSuccessConditions(const std::function<void (boost::shared_ptr<ConditionBase>)>& visitor);
    void applyVisitorForFailureConditions(const std::function<void (boost::shared_ptr<ConditionBase>)>& visitor);

  private:
    condition_type loadPlugin(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr);

    std::vector<condition_type> success_conditions_,
                                failure_conditions_;

    ConditionVisualizer visualizer;
  };
}  // namespace scenario_conditions

#endif  // SCENARIO_CONDITION_CONDITION_MANAGER_H_INCLUDED
