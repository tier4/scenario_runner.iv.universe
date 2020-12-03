#ifndef SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED
#define SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED


#include <scenario_expression/expression.hpp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_sequence/sequence.hpp>
#include <scenario_utility/scenario_utility.hpp>

#include <yaml-cpp/yaml.h>

#include <memory>
#include <queue>

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

