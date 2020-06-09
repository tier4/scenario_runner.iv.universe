#include <scenario_sequence/sequence_manager.h>

namespace scenario_sequence
{

SequenceManager::SequenceManager(
  const scenario_expression::Environment& env, const YAML::Node& sequences)
  : env_ { env }
{
  for (const auto& each : sequences)
  {
    sequences_.emplace(each["Sequence"], env_.api, env_.entities);
  }
}

simulation_is SequenceManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  if (not sequences_.empty())
  {
    ROS_INFO_STREAM("\e[1;32m  Act:\e[0m");

    switch (const auto result {sequences_.front().update(env_.intersections)})
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

