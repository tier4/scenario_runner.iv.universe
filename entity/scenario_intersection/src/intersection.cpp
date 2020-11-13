#include <scenario_intersection/intersection.h>

namespace scenario_intersection
{

Intersection::Intersection(
  const YAML::Node& script,
  rclcpp::Logger & logger
  // const std::shared_ptr<ScenarioAPI>& simulator
)
  : script_(script)
  // , simulator_ {simulator}
{
  if (const auto ids {script_["TrafficLightId"]})
  {
    for (const auto& each : ids)
    {
      ids_.emplace_back(each.as<std::size_t>());
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(
      logger, "Each element of node 'Intersection' requires hash 'TrafficLightId'.");
  }

  if (const auto controls {script_["Control"]})
  {
    for (const auto& each : controls)
    {
      if (const auto& state_name {each["StateName"]})
      {
        change_to_.emplace(state_name.as<std::string>(), Controller(each, logger));
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger, "Each element of node 'Control' requires hash 'StateName'.");
      }
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger, "Each element of node 'Intersection' requires hash 'Control'.");
  }
}

bool Intersection::change_to(const std::string& the_state)
{
  // NOTE: Any unspecified state names are treated as "Blank" state
  // return change_to_[current_state_ = the_state](*simulator_);
  return false;
}

const std::vector<std::size_t>& Intersection::ids() const
{
  return ids_;
}

simulation_is Intersection::update()
{
  return simulation_is::ongoing;
}

} // namespace scenario_intersection
