#ifndef INCLUDED_ACTION_PLUGINS_ENABLE_ACTION_H
#define INCLUDED_ACTION_PLUGINS_ENABLE_ACTION_H

#include <scenario_actions/entity_action_base.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_logger/logger.h>
#include <scenario_utility/scenario_utility.h>

namespace action_plugins
{

class EnableAction
  : public scenario_actions::EntityActionBase
{
  std::string name_;

  boost::optional<bool> automatic_emergency_braking_;

public:
  EnableAction();

  void configure(
    const YAML::Node&,
    const std::vector<std::string>,
    const std::shared_ptr<ScenarioAPI>&) override;

  void run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    override;
};

} // namespace action_plugins

#endif // INCLUDED_ACTION_PLUGINS_ENABLE_ACTION_H
