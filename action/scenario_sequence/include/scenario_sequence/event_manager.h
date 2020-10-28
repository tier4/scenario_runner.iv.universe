#ifndef SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED
#define SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED

#include <queue>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <scenario_expression/expression.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_sequence/event.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_sequence
{

class EventManager
{
  scenario_expression::Context context_;

  std::queue<scenario_sequence::Event> events_;

public:
  EventManager(const scenario_expression::Context&, const YAML::Node&);

  state_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);

  state_is currently;
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED

