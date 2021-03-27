#ifndef SCENARIO_SEQUENCE_SEQUENCE_H_INCLUDED
#define SCENARIO_SEQUENCE_SEQUENCE_H_INCLUDED

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

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

  scenario_expression::Expression start_condition_;

  bool ignited_;

public:
  Sequence(const scenario_expression::Context&, const YAML::Node&);

  const auto& name() const noexcept
  {
    return name_;
  }

  void touch();

  state_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);

  state_is currently;
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_SEQUENCE_H_INCLUDED

