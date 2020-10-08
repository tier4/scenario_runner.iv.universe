#include <scenario_sequence/sequence_manager.h>

namespace scenario_sequence
{

SequenceManager::SequenceManager(
  const scenario_expression::Context& context, const YAML::Node& sequences)
  : context_ { context }
{
  for (const auto& each : sequences)
  {
    sequences_.emplace(context, each["Sequence"]);
  }
}

simulation_is SequenceManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  if (not sequences_.empty())
  {
    switch (const auto result { sequences_.front().update(context_.intersections_pointer()) })
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
    return simulation_is::succeeded;
  }
}

} // namespace scenario_sequence

