#include <scenario_sequence/event.h>

namespace scenario_sequence
{

Event::Event(
  const scenario_expression::Context& context,
  const YAML::Node& event_definition)
  : context_ { context }
  , name_ { event_definition["Name"].as<std::string>() }
  , ignited_ { false }
  , currently { state_is::sleeping }
{
  for (const auto& each : event_definition["Actors"])
  {
    actors_.push_back(each.as<std::string>());
  }

  action_manager_ =
    std::make_shared<scenario_actions::ActionManager>(
      event_definition["Actions"], actors_, context.api_pointer());

  if (const auto condition { event_definition["Condition"] })
  {
    condition_ = scenario_expression::read(context_, condition);
  }
  else // NOTE: If Condition unspecified, the sequence starts unconditionally.
  {
    ignited_ = true;
  }
}

void Event::touch() const
{
  context_.json << (indent++) << "{\n";
  context_.json << indent << "Name: " << std::quoted(name_) << ",\n";
  context_.json << (indent++) << "Conditions: [\n";
  context_.json << condition_;
  context_.json << (--indent) << "],\n";
  context_.json << indent << "State: " << currently << ",\n";
  context_.json << (--indent) << "},\n";
}

state_is Event::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  context_.json << (indent++) << "{\n";
  context_.json << indent << "Name: " << std::quoted(name_) << ",\n";

  context_.json << (indent++) << "Conditions: [\n";
  context_.json << condition_;
  context_.json << (--indent) << "],\n";

  ignited_ = condition_.evaluate(context_);

  if (ignited_)
  {
    (*action_manager_).run(context_.intersections_pointer());
    currently = state_is::finished;
  }
  else
  {
    currently = state_is::running;
  }

  context_.json << indent << "State: " << currently << ",\n";
  context_.json << (--indent) << "},\n";

  return currently;
}

std::ostream& operator <<(std::ostream& os, const state_is& currently)
{
  switch (currently)
  {
  case state_is::sleeping:
    return os // << "\x1b[33m"
              << std::quoted("NotRunning")
              // << "\x1b[0m"
              ;

  case state_is::running:
    return os // << "\x1b[32m"
              << std::quoted("Running")
              // << "\x1b[0m"
              ;

  case state_is::finished:
    return os // << "\x1b[31m"
              << std::quoted("Finished")
              // << "\x1b[0m"
              ;
  }
}

std::ostream& operator <<(std::ostream& os, const state_color& datum)
{
  os << "\x1b[0m";

  switch (datum.value)
  {
  case state_is::finished:
    return os << "\x1b[2m";

  case state_is::running:
    return os << "\x1b[32m";

  case state_is::sleeping:
    return os;
  }
}
} // namespace scenario_sequence

