#include <scenario_sequence/event_manager.h>

namespace scenario_sequence
{

EventManager::EventManager(
  const YAML::Node& events_definition,
  const std::shared_ptr<ScenarioAPI>& simulator,
  const std::shared_ptr<scenario_entities::EntityManager>& entity_manager)
  : events_definition_ {events_definition}
  , simulator_ {simulator}
  , entity_manager_ {entity_manager}
{
  for (const auto& each : events_definition_)
  {
    events_.emplace(each, simulator_, entity_manager_);
  }
}

simulation_is EventManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>& intersection_manager)
{
  if (not events_.empty())
  {
    switch (const auto result {events_.front().update(intersection_manager)})
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

