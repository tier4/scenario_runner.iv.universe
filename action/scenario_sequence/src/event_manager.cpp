#include <scenario_sequence/event_manager.h>

namespace scenario_sequence
{

EventManager::EventManager(
  const scenario_expression::Context& context,
  const YAML::Node& events_definition)
  : context_ { context }
  , currently { state_is::sleeping }
{
  for (const auto& each : events_definition)
  {
    events_.emplace_back(context, each);
  }

  cursor = std::begin(events_);
}

state_is EventManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  if (cursor != std::end(events_))
  {
    switch (currently = (*cursor).update(context_.intersections_pointer()))
    {
    case state_is::finished:
      ++cursor;
      return currently = state_is::running;

    default:
      return currently;
    }
  }
  else
  {
    return currently = state_is::finished;
  }
}

} // namespace scenario_sequence
