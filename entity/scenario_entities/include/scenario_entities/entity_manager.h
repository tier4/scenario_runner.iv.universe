#ifndef SCENARIO_ENTITIES_ENTITY_MANAGER_H_INCLUDED
#define SCENARIO_ENTITIES_ENTITY_MANAGER_H_INCLUDED

#include <pluginlib/class_loader.h>

#include <scenario_api/scenario_api_core.h>
#include <scenario_entities/entity_base.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_entities
{

class EntityManager
{
public:
  EntityManager(
    const YAML::Node& node,
    const std::shared_ptr<ScenarioAPI>& api_ptr);

  bool setStory(const YAML::Node & story);

  bool initialize();

  simulation_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager> &);

private:
  bool loadPlugin(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr);

  std::vector<boost::shared_ptr<scenario_entities::EntityBase>> entities_;

  const std::shared_ptr<ScenarioAPI> api_ptr_;
};

}  // namespace scenario_entities

#endif // SCENARIO_ENTITIES_ENTITY_MANAGER_H_INCLUDED

