#include <scenario_sequence/sequence_manager.h>

namespace scenario_sequence
{

SequenceManager::SequenceManager(
  const YAML::Node& sequences,
  const std::shared_ptr<ScenarioAPI>& simulator,
  const std::shared_ptr<scenario_entities::EntityManager>& entity_manager)
  : sequences_definition_ {sequences}
  , simulator_ {simulator}
  , entity_manager_ {entity_manager}
{
  for (const auto& each : sequences_definition_)
  {
    sequences_.emplace(each["Sequence"], simulator_, entity_manager_);
  }
}

simulation_is SequenceManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>& intersection_manager)
{
  if (not sequences_.empty())
  {
    ROS_INFO_STREAM("\e[1;32m  Act:\e[0m");

    switch (const auto result {sequences_.front().update(intersection_manager)})
    {
    case simulation_is::succeeded:
      sequences_.pop();
      return simulation_is::ongoing;

    default:
      return result;
    }
  }
  else
  {
    ROS_INFO_STREAM("\e[1;32m  Act: \e[1;36mExhausted\e[0m");
    return simulation_is::succeeded;
  }
}

} // namespace scenario_sequence

