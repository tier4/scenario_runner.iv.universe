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

void EventManager::touch() const
{
  context_.json << (indent++) << "Events: [\n";

  for (auto iter { std::begin(events_) }; iter != cursor; ++iter)
  {
    (*iter).touch();
  }

  if (cursor != std::end(events_))
  {
    (*cursor).touch();

    for (auto iter { std::next(cursor) }; iter != std::end(events_); ++iter)
    {
      (*iter).touch();
    }
  }

  context_.json << (--indent) << "],\n";
}

state_is EventManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  context_.json << (indent++) << "Events: [\n";

  for (auto iter { std::begin(events_) }; iter != cursor; ++iter)
  {
    (*iter).touch();
  }

  context_.current_event = (*cursor).name();

  switch (currently = (*cursor).update(context_.intersections_pointer()))
  {
  case state_is::finished:
    for (auto iter { ++cursor }; iter != std::end(events_); ++iter)
    {
      (*iter).touch();
    }
    break;

  default:
    for (auto iter { std::next(cursor) }; iter != std::end(events_); ++iter)
    {
      (*iter).touch();
    }
    break;
  }

  context_.json << (--indent) << "],\n";

  return currently;
}

} // namespace scenario_sequence
