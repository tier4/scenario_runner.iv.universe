#ifndef ACION_PLUGINS_LANE_CHANGE_ACTION_H_INCLUDED
#define ACION_PLUGINS_LANE_CHANGE_ACTION_H_INCLUDED

#include <scenario_actions/entity_action_base.hpp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_utility/scenario_utility.hpp>

namespace action_plugins
{

class LaneChangeAction
  : public scenario_actions::EntityActionBase
{
  int target_lanelet_;

public:
  LaneChangeAction();

  void configure(
    const YAML::Node& node,
    const std::vector<std::string> actors,
    const std::shared_ptr<ScenarioAPI>& api_ptr) override;

  auto run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    -> void override;
};

} // namespace action_plugins

#endif // ACION_PLUGINS_LANE_CHANGE_ACTION_H_INCLUDED

