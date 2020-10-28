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
  std::cout << "  Sequences: [" << std::endl;

  std::cout << "\x1b[2m";
  for (auto iter { std::begin(sequences_) }; iter != cursor; ++iter)
  {
    (*iter).touch();
  }
  std::cout << "\x1b[0m";

  if (cursor != std::end(sequences_))
  {
    switch (currently = (*cursor).update(context_.intersections_pointer()))
    {
    case state_is::finished:
      for (auto iter { ++cursor }; iter != std::end(sequences_); ++iter)
      {
        (*iter).touch();
      }
      break;

    default:
      for (auto iter { std::next(cursor) }; iter != std::end(sequences_); ++iter)
      {
        (*iter).touch();
      }
      break;
    }
  }
  else
  {
    currently = state_is::finished;
  }

  std::cout << "  ]," << std::endl;
}

} // namespace scenario_sequence

