#include <scenario_sequence/sequence.h>

namespace scenario_sequence
{

Sequence::Sequence(
  const scenario_expression::Context& context,
  const YAML::Node& sequence_definition)
  : context_ { context }
  , name_ {sequence_definition["Name"].as<std::string>()}
  , ignited_ {false}
{
  ROS_INFO_STREAM("\e[1;32m    - Sequence:\e[0m");
  ROS_INFO_STREAM("\e[1;32m        Name: " << name_ << "\e[0m");

  ROS_INFO_STREAM("\e[1;32m        Events:\e[0m");
  event_manager_ =
    std::make_shared<scenario_sequence::EventManager>(
      context, sequence_definition["Events"]);

  ROS_INFO_STREAM("\e[1;32m        StartCondition:\e[0m");
  {
    if (const auto start_condition { sequence_definition["StartCondition"] })
    {
      start_condition_ = scenario_expression::read(context_, start_condition);
    }
    else // NOTE: If StartCondition unspecified, the sequence starts unconditionally.
    {
      ignited_ = true;
    }
  }
}

simulation_is Sequence::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  ROS_INFO_STREAM("\e[1;32m    - Sequence:\e[0m");
  ROS_INFO_STREAM("\e[1;32m        Name: " << name_ << "\e[0m");
  ROS_INFO_STREAM("\e[1;32m        StartCondition:\e[0m");

  if (ignited_ = start_condition_.evaluate(context_))
  {
    ROS_INFO_STREAM("\e[1;32m          Ignited\e[0m");
    ROS_INFO_STREAM("\e[1;32m        Events:\e[0m");
    return (*event_manager_).update(context_.intersections);
  }
  else
  {
    ROS_INFO_STREAM("\e[1;32m        Events: \e[36mPending\e[0m");
    return simulation_is::ongoing;
  }
}

} // namespace scenario_sequence

