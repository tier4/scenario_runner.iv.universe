#ifndef SCENARIO_SEQUENCE_EVENT_H_INCLUDED
#define SCENARIO_SEQUENCE_EVENT_H_INCLUDED

#include <scenario_actions/action_manager.h>
#include <scenario_conditions/condition_manager.h>
#include <scenario_entities/entity_manager.h>
#include <scenario_expression/expression.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_sequence
{

class Event
{
  scenario_expression::Context context_;

  const std::string name_;

  std::vector<std::string> actors_;

  std::shared_ptr<scenario_actions::ActionManager> action_manager_;

  scenario_expression::Expression condition_;

  using condition_pointer
    = boost::shared_ptr<scenario_conditions::ConditionBase>;

  // std::vector<condition_pointer> conjunctional_conditions_,
  //                                disjunctional_conditions_;
  bool ignited_;

  condition_pointer load(const YAML::Node& node) const;

public:
  Event(const scenario_expression::Context&, const YAML::Node&);

  simulation_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_EVENT_H_INCLUDED

