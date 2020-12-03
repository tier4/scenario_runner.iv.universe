#ifndef SCENARIO_ACTIONS_ACTION_MANAGER_H_INCLUDED
#define SCENARIO_ACTIONS_ACTION_MANAGER_H_INCLUDED

#include <scenario_actions/entity_action_base.hpp>
#include <scenario_api/scenario_api_core.hpp>
#include <scenario_intersection/intersection_manager.hpp>

#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <vector>


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

  std::vector<std::shared_ptr<EntityActionBase>> actions_;

  void loadPlugin(const YAML::Node& node);
};

}  // namespace scenario_actions

#endif // SCENARIO_ACTIONS_ACTION_MANAGER_H_INCLUDED
