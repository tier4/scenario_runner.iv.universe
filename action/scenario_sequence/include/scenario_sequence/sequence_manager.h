#ifndef SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED
#define SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED

#include <queue>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <scenario_entities/entity_manager.h>
#include <scenario_expression/expression.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_sequence/sequence.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_sequence
{

class SequenceManager
{
  std::queue<scenario_sequence::Sequence> sequences_;

  scenario_expression::Context context_;

public:
  SequenceManager(const scenario_expression::Context&, const YAML::Node&);

  simulation_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED

