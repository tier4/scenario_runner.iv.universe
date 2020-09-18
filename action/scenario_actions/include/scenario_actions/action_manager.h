#ifndef SCENARIO_ACTIONS_ACTION_MANAGER_H_INCLUDED
#define SCENARIO_ACTIONS_ACTION_MANAGER_H_INCLUDED

#include <memory>
#include <vector>

#include <pluginlib/class_loader.h>

#include <yaml-cpp/yaml.h>

#include <scenario_actions/entity_action_base.h>
#include <scenario_api/scenario_api_core.h>
#include <scenario_intersection/intersection_manager.h>

namespace scenario_actions
{

class ActionManager
{
public:
  ActionManager(
    const YAML::Node& node,
    const std::vector<std::string>& actors,
    const std::shared_ptr<ScenarioAPI>& api_ptr);

  auto run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    -> void;

private:
  const YAML::Node actions_node_;
  const std::vector<std::string> actors_;

  const std::shared_ptr<ScenarioAPI> api_ptr_;

  std::vector<boost::shared_ptr<EntityActionBase>> actions_;

  bool loadPlugin(const YAML::Node& node);
};

}  // namespace scenario_actions

#endif // SCENARIO_ACTIONS_ACTION_MANAGER_H_INCLUDED
