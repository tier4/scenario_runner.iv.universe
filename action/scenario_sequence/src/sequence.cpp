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

void Sequence::dummy()
{
  std::cout << "    {" << std::endl;
  std::cout << "      Name: " << name_ << "," << std::endl;
  std::cout << "      StartConditions: [" << std::endl;
  std::cout << "        TODO," << std::endl;
  std::cout << "      ]," << std::endl;
  std::cout << "      State: " << currently << "," << std::endl;
  std::cout << "    }," << std::endl;
}

state_is Sequence::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  std::cout << "    {" << std::endl;
  std::cout << "      Name: " << name_ << "," << std::endl;

  std::cout << "      StartConditions: [" << std::endl;
  ignited_ = start_condition_.evaluate(context_);
  std::cout << "      ]," << std::endl;

  if (ignited_)
  {
    currently = (*event_manager_).update(context_.intersections_pointer());
  }
  else
  {
    currently = state_is::running;
  }

  std::cout << "      State: " << currently << "," << std::endl;
  std::cout << "    }," << std::endl;

  return currently;
}

} // namespace scenario_sequence

