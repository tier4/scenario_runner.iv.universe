#ifndef SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED
#define SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED


#include <scenario_expression/expression.hpp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_sequence/event.hpp>
#include <scenario_utility/scenario_utility.hpp>

#include <yaml-cpp/yaml.h>

#include <memory>
#include <queue>

namespace scenario_sequence
{

class EventManager
{
  scenario_expression::Context context_;

  std::queue<scenario_sequence::Event> events_;

public:
  EventManager(const scenario_expression::Context&, const YAML::Node&);

  simulation_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED

