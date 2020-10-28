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
}

state_is SequenceManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  std::cout << "  Sequences: [" << std::endl;

  if (not sequences_.empty())
  {
    switch (currently = sequences_.front().update(context_.intersections_pointer()))
    {
    case state_is::finished:
      sequences_.pop_front();

    default:
      break;
    }
  }
  else
  {
    currently = state_is::running;
  }

  for (auto iter { std::next(std::begin(sequences_)) }; iter != std::end(sequences_); ++iter)
  {
    std::cout << "\x1b[2m";
    (*iter).dummy();
    std::cout << "\x1b[0m";
  }

  std::cout << "  ]," << std::endl;
}

} // namespace scenario_sequence

