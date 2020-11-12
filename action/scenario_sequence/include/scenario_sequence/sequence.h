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

  template <typename... Ts>
  decltype(auto) current_event_name(Ts&&... xs) const
  {
    return (*event_manager_).current_event_name(std::forward<decltype(xs)>(xs)...);
  }

  auto property() const
  {
    boost::property_tree::ptree result {};

    result.put("Name", name());
    result.put("State", boost::lexical_cast<std::string>(currently));
    result.add_child("StartConditions", start_condition_.property());
    result.add_child("Events", (*event_manager_).property());

    return result;
  }

  state_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);

  state_is currently;
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_SEQUENCE_H_INCLUDED

