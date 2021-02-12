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

#include "scenario_intersection/intersection_manager.hpp"

#include <memory>
#include <string>

namespace scenario_intersection
{

IntersectionManager::IntersectionManager(
  const YAML::Node & node,
  const std::shared_ptr<ScenarioAPI> & simulator)
: node_{node},
  logger_(rclcpp::get_logger("scenario_intersection_manager")),
  simulator_{simulator}
{
  for (const auto & intersection : node_) {
    if (const auto name {intersection["Name"]}) {
      intersections_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(name.as<std::string>()),
        std::forward_as_tuple(intersection, simulator_));
    } else {
      RCLCPP_ERROR_STREAM(logger_, "Missing key 'Name' at element of 'Intersection'.");
    }
  }
}

bool IntersectionManager::initialize(const YAML::Node & intersections)
{
  if (intersections) {
    return
      std::all_of(
      intersections.begin(), intersections.end(),
      [this](const auto & intersection)
      {
        if (const auto name {intersection["Name"]}) {
          if (const auto the_state {intersection["InitialState"]}) {
            return intersections_.at(name.template as<std::string>()).change_to(
              the_state.template
              as<std::string>());
          } else {
            return intersections_.at(name.template as<std::string>()).change_to("Blank");
          }
        } else {
          RCLCPP_ERROR_STREAM(
            logger_,
            "Missing key 'Name' at element of 'Story.Init.Intersection'.");
          return false;
        }
      });
  } else {
    return true;  // Story.Init.Intersection is optional.
  }
}

bool IntersectionManager::change(
  const std::string & target_intersection,
  const std::string & the_state)
try
{
  return intersections_.at(target_intersection).change_to(the_state);
} catch (const std::out_of_range &) {
  RCLCPP_ERROR_STREAM(
    logger_,
    "You are trying to change state of unspecified intersection '" << target_intersection <<
      "'.");
  return false;
}

simulation_is IntersectionManager::update()
{
  return simulation_is::ongoing;
}
}  // namespace scenario_intersection
