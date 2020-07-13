#include <scenario_intersection/intersection_manager.h>

namespace scenario_intersection
{

IntersectionManager::IntersectionManager(
  const YAML::Node& node,
  const std::shared_ptr<ScenarioAPI>& simulator)
  : node_ {node}
  , simulator_ {simulator}
{
  for (const auto& intersection : node_)
  {
    if (const auto name {intersection["Name"]})
    {
      intersections_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(name.as<std::string>()),
        std::forward_as_tuple(intersection, simulator_));
    }
    else
    {
      ROS_ERROR_STREAM("Missing key 'Name' at element of 'Intersection'.");
    }
  }
}

bool IntersectionManager::initialize(const YAML::Node& intersections)
{
  if (intersections)
  {
    return
      std::all_of(
        intersections.begin(), intersections.end(),
        [this](const auto& intersection)
        {
          if (const auto name {intersection["Name"]})
          {
            if (const auto the_state {intersection["InitialState"]})
            {
              return intersections_.at(name.template as<std::string>()).change_to(the_state.template as<std::string>());
            }
            else
            {
              return intersections_.at(name.template as<std::string>()).change_to("Blank");
            }
          }
          else
          {
            ROS_ERROR_STREAM("Missing key 'Name' at element of 'Story.Init.Intersection'.");
            return false;
          }
        });
  }
  else
  {
    return true; // Story.Init.Intersection is optional.
  }
}

bool IntersectionManager::change(
  const std::string& target_intersection,
  const std::string& the_state)
try
{
  return intersections_.at(target_intersection).change_to(the_state);
}
catch (const std::out_of_range&)
{
  ROS_ERROR_STREAM("You trying to change state of unspecified intersection '" << target_intersection << "'.");
  return false;
}

simulation_is IntersectionManager::update(const ros::Time& now)
{
  return simulation_is::ongoing;
}

} // namespace scenario_intersection

