#ifndef SCENARIO_INTERSECTION_INTERSECTON_MANAGER_H_INCLUDED
#define SCENARIO_INTERSECTION_INTERSECTON_MANAGER_H_INCLUDED

#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include <scenario_api/scenario_api_core.h>
#include <scenario_intersection/intersection.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_intersection
{

class IntersectionManager
{
  const YAML::Node node_;

  const std::shared_ptr<ScenarioAPI> simulator_;

  std::unordered_map<std::string, scenario_intersection::Intersection> intersections_;

public:
  IntersectionManager(
    const YAML::Node&,
    const std::shared_ptr<ScenarioAPI>&);

  bool initialize(const YAML::Node&);

  bool change(const std::string&, const std::string&);

  simulation_is update(const ros::Time&);

  template <typename... Ts>
  constexpr decltype(auto) at(Ts&&... xs) const
  {
    return intersections_.at(std::forward<decltype(xs)>(xs)...);
  }
};

} // namespace scenario_intersection

#endif // SCENARIO_INTERSECTION_INTERSECTON_MANAGER_H_INCLUDED

