#include <scenario_sequence/sequence.h>

namespace scenario_sequence
{

Sequence::Sequence(
  const scenario_expression::Context& context,
  const YAML::Node& sequence_definition)
  : context_ { context }
  , name_ { sequence_definition["Name"].as<std::string>() }
  , ignited_ { false }
  , currently { state_is::sleeping }
{
  event_manager_ =
    std::make_shared<scenario_sequence::EventManager>(
      context, sequence_definition["Events"]);

  if (const auto start_condition { sequence_definition["StartCondition"] })
  {
    start_condition_ = scenario_expression::read(context_, start_condition);
  }
  else // NOTE: If StartCondition unspecified, the sequence starts unconditionally.
  {
    ignited_ = true;
  }
}

void Sequence::touch()
{
  std::cout << (indent++) << "{\n";
  std::cout << indent << "Name: " << std::quoted(name_) << ",\n";
  std::cout << (indent++) << "StartConditions: [\n";
  std::cout << indent << start_condition_ << "\n";
  std::cout << (--indent) << "],\n";

  (*event_manager_).touch();

  std::cout << indent << "State: " << currently << ",\n";
  std::cout << (--indent) << "},\n";
}

state_is Sequence::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  std::cout << (indent++) << "{\n";
  std::cout << indent << "Name: " << std::quoted(name_) << ",\n";
  std::cout << (indent++) << "StartConditions: [\n";
  std::cout << indent << start_condition_ << "\n";
  std::cout << (--indent) << "],\n";

  ignited_ = start_condition_.evaluate(context_);

  if (ignited_)
  {
    currently = (*event_manager_).update(context_.intersections_pointer());
  }
  else
  {
    (*event_manager_).touch();
    currently = state_is::running;
  }

  std::cout << indent << "State: " << currently << ",\n";
  std::cout << (--indent) << "},\n";

  return currently;
}

} // namespace scenario_sequence
