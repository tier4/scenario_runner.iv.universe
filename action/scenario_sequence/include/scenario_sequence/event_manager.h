#ifndef SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED
#define SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED

#include <list>

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

  std::list<scenario_sequence::Event> events_;

  decltype(events_)::iterator cursor;

public:
  EventManager(const scenario_expression::Context&, const YAML::Node&);

  const auto& current_event_name() const
  {
    if (cursor != std::end(events_))
    {
      return (*cursor).name();
    }
    else
    {
      static const std::string it { "" };
      return it;
    }
  }

  auto property() const
  {
    boost::property_tree::ptree result {};

    if (not events_.empty())
    {
      for (const auto& each : events_)
      {
        result.push_back(std::make_pair("", each.property()));
      }
    }
    else
    {
      result.push_back(std::make_pair("", boost::property_tree::ptree())); // XXX HACK
    }

    return result;
  }

  state_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);

  state_is currently;
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED

