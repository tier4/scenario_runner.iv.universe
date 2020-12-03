#ifndef CONDITION_PLUGINS_REACH_POSITION_CONDITION_H_INCLUDED
#define CONDITION_PLUGINS_REACH_POSITION_CONDITION_H_INCLUDED

#include <geometry_msgs/msg/pose.hpp>
#include <scenario_conditions/condition_base.hpp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_logger/logger.hpp>
#include <scenario_utility/scenario_utility.hpp>

namespace condition_plugins
{

class ReachPositionCondition : public scenario_conditions::ConditionBase
{
public:
  ReachPositionCondition();
  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) override;

private:
  geometry_msgs::msg::Pose target_pose_;

  std::string trigger_, shift_;

  float tolerance_;
};

}  // namespace condition_plugins

#endif  // CONDITION_PLUGINS_REACH_POSITION_CONDITION_H_INCLUDED
