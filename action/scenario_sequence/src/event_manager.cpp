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
    events_.emplace(context, each);
  }
}

state_is EventManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  std::cout << "      Events: [" << std::endl;

  switch (currently = events_.front().update(context_.intersections_pointer()))
  {
  case state_is::finished:
    events_.pop();

  default:
    break;
  }

  std::cout << "      ]," << std::endl;

  return currently;
}

} // namespace scenario_sequence

