#include <scenario_sequence/event.h>

namespace scenario_sequence
{

Event::Event(
  const scenario_expression::Context& context,
  const YAML::Node& event_definition)
  : context_ { context }
  , name_ {event_definition["Name"].as<std::string>()}
  , ignited_ {false}
{
  for (const auto& each : event_definition["Actors"])
  {
    actors_.push_back(each.as<std::string>());
  }

  action_manager_ =
    std::make_shared<scenario_actions::ActionManager>(
      event_definition["Actions"], actors_, context.api);

  if (const auto condition { event_definition["Condition"] })
  {
    condition_ = scenario_expression::read(context_, condition);
  }
  else // NOTE: If Condition unspecified, the sequence starts unconditionally.
  {
    ignited_ = true;
  }
}

simulation_is Event::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  if (ignited_ = condition_.evaluate(context_))
  {
    (*action_manager_).run(context_.intersections);
    return simulation_is::succeeded;
  }
  else
  {
    return simulation_is::ongoing;
  }
}

} // namespace scenario_sequence

