#ifndef SCENARIO_SEQUENCE_EVENT_H_INCLUDED
#define SCENARIO_SEQUENCE_EVENT_H_INCLUDED

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <scenario_actions/action_manager.h>
#include <scenario_entities/entity_manager.h>
#include <scenario_expression/expression.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_sequence
{

enum class state_is
{
  sleeping, running, finished,
};

std::ostream& operator <<(std::ostream&, const state_is&);

class Event
{
  scenario_expression::Context context_;

  const std::string name_;

  std::vector<std::string> actors_;

  std::shared_ptr<scenario_actions::ActionManager> action_manager_;

  scenario_expression::Expression condition_;

  bool ignited_;

public:
  Event(const scenario_expression::Context&, const YAML::Node&);

  const auto& name() const noexcept
  {
    return name_;
  }

  boost::property_tree::ptree property() const
  {
    boost::property_tree::ptree result {};

    result.put("Name", name());
    result.put("State", boost::lexical_cast<std::string>(currently));
    result.add_child("Conditions", condition_.property());

    return result;
  }

  state_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);

  state_is currently;
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_EVENT_H_INCLUDED

