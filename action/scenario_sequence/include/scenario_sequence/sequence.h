#ifndef SCENARIO_SEQUENCE_SEQUENCE_H_INCLUDED
#define SCENARIO_SEQUENCE_SEQUENCE_H_INCLUDED

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <scenario_conditions/condition_manager.h>
#include <scenario_entities/entity_manager.h>
#include <scenario_expression/expression.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_sequence/event_manager.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_sequence
{

class Sequence
{
  scenario_expression::Context context_;

  const std::string name_;

  std::shared_ptr<scenario_sequence::EventManager> event_manager_;

  using condition_pointer
    = boost::shared_ptr<scenario_conditions::ConditionBase>;

  scenario_expression::Expression start_condition_;
  // std::vector<condition_pointer> conjunctional_conditions_,
  //                                disjunctional_conditions_;
  bool ignited_;

  condition_pointer load(const YAML::Node& node) const;

public:
  Sequence(const scenario_expression::Context&, const YAML::Node&);

  simulation_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_SEQUENCE_H_INCLUDED

