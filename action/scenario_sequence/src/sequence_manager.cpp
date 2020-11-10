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
  context_.json << (indent++) << "Sequences: [\n";

  for (auto iter { std::begin(sequences_) }; iter != cursor; ++iter)
  {
    (*iter).touch();
    context_.json << (std::next(iter) != std::end(sequences_) ? ",\n" : "\n");
  }

  if (cursor != std::end(sequences_))
  {
    currently = (*cursor).update(context_.intersections_pointer());
    context_.json << (std::next(cursor) != std::end(sequences_) ? ",\n" : "\n");

    switch (currently)
    {
    case state_is::finished:
      for (auto iter { ++cursor }; iter != std::end(sequences_); ++iter)
      {
        (*iter).touch();
        context_.json << (std::next(iter) != std::end(sequences_) ? ",\n" : "\n");
      }
      break;

    default:
      for (auto iter { std::next(cursor) }; iter != std::end(sequences_); ++iter)
      {
        (*iter).touch();
        context_.json << (std::next(iter) != std::end(sequences_) ? ",\n" : "\n");
      }
      break;
    }
  }
  else
  {
    currently = state_is::finished;
  }

  context_.json << (--indent) << "],\n";

  return currently;
}

} // namespace scenario_sequence
