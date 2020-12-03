#ifndef INCLUDED_ACION_PLUGINS_CHANGE_SIGNAL_ACTION_H
#define INCLUDED_ACION_PLUGINS_CHANGE_SIGNAL_ACTION_H

#include <pluginlib/class_list_macros.hpp>

#include <scenario_actions/entity_action_base.hpp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_utility/scenario_utility.hpp>

namespace action_plugins
{

class ChangeSignalAction
  : public scenario_actions::EntityActionBase
{
  std::string target_intersection_, state_;

public:
  ChangeSignalAction()
    : EntityActionBase {"ChangeSignal"}
  {}

  void configure(
    const YAML::Node& node,
    const std::vector<std::string> actors,
    const std::shared_ptr<ScenarioAPI>& api_ptr)
    override;

  void run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    override;
};

} // namespace action_plugins

#endif // INCLUDED_ACION_PLUGINS_CHANGE_SIGNAL_ACTION_H
