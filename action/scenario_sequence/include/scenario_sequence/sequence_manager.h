#ifndef SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED
#define SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED

#include <queue>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <scenario_entities/entity_manager.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_sequence/sequence.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_sequence
{

class SequenceManager
{
  const YAML::Node sequences_definition_;

  const std::shared_ptr<ScenarioAPI> simulator_;
  const std::shared_ptr<scenario_entities::EntityManager> entity_manager_;

  std::queue<scenario_sequence::Sequence> sequences_;

public:
  SequenceManager(
    const YAML::Node&,
    const std::shared_ptr<ScenarioAPI>&,
    const std::shared_ptr<scenario_entities::EntityManager>&);

  simulation_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED

