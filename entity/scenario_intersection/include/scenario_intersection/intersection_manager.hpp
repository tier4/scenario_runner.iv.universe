// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SCENARIO_INTERSECTION__INTERSECTION_MANAGER_HPP_
#define SCENARIO_INTERSECTION__INTERSECTION_MANAGER_HPP_

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <utility>
#include <unordered_map>

#include "rclcpp/logging.hpp"
#include "rclcpp/logger.hpp"

#include "scenario_api/scenario_api_core.hpp"
#include "scenario_intersection/intersection.hpp"
#include "scenario_utility/scenario_utility.hpp"

namespace scenario_intersection
{

class IntersectionManager
{
  const YAML::Node node_;

  rclcpp::Logger logger_;

  const std::shared_ptr<ScenarioAPI> simulator_;

  std::unordered_map<std::string, scenario_intersection::Intersection> intersections_;

public:
  IntersectionManager(
    const YAML::Node &,
    const std::shared_ptr<ScenarioAPI> &
  );

  bool initialize(const YAML::Node &);

  bool change(const std::string &, const std::string &);

  simulation_is update();

  template<typename ... Ts>
  constexpr decltype(auto) at(Ts && ... xs) const
  {
    return intersections_.at(std::forward<decltype(xs)>(xs)...);
  }
};

}  // namespace scenario_intersection
#endif  // SCENARIO_INTERSECTION__INTERSECTION_MANAGER_HPP_
