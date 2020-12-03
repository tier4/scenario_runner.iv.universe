#include <scenario_sequence/event_manager.hpp>

namespace scenario_sequence
{

EventManager::EventManager(
  const scenario_expression::Context& context,
  const YAML::Node& events_definition)
  : context_ { context }
{
  for (const auto& each : events_definition)
  {
    events_.emplace(context, each);
  }
}

simulation_is EventManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  if (not events_.empty())
  {
    switch (const auto result { events_.front().update(context_.intersections_pointer()) })
    {
    case simulation_is::succeeded:
      events_.pop();
      return simulation_is::ongoing;

    default:
      return result;
    }
  }
  else
  {
    return simulation_is::succeeded;
  }
}

} // namespace scenario_sequence

