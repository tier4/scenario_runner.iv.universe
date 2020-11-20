#ifndef SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED
#define SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED

#include <list>

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
  std::list<scenario_sequence::Sequence> sequences_;

  scenario_expression::Context context_;

  // NOTE: Adding, removing and moving the elements within the list or across
  // several lists does not invalidate the iterators or references.
  decltype(sequences_)::iterator cursor;

public:
  SequenceManager(const scenario_expression::Context&, const YAML::Node&);

  const auto& current_sequence_name() const
  {
    if (cursor != std::end(sequences_))
    {
      return (*cursor).name();
    }
    else
    {
      static const std::string it { "" };
      return it;
    }
  }

  const auto& current_event_name() const
  {
    if (cursor != std::end(sequences_))
    {
      return (*cursor).current_event_name();
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

    if (not sequences_.empty())
    {
      for (const auto& each : sequences_)
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

#endif // SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED

