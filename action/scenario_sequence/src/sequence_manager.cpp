#include <scenario_sequence/sequence_manager.h>

namespace scenario_sequence
{

SequenceManager::SequenceManager(
  const scenario_expression::Context& context, const YAML::Node& sequences)
  : context_ { context }
  , currently { state_is::sleeping }
{
  for (const auto& each : sequences)
  {
    sequences_.emplace_back(context, each["Sequence"]);
  }

  cursor = std::begin(sequences_);
}

state_is SequenceManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  if (cursor != std::end(sequences_))
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
