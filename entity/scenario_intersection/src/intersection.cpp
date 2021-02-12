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

#include "scenario_intersection/intersection.hpp"

#include <memory>
#include <string>
#include <vector>

namespace scenario_intersection
{

Intersection::Intersection(
  const YAML::Node & script,
  const std::shared_ptr<ScenarioAPI> & simulator
)
: script_{script},
  simulator_{simulator}
{
  rclcpp::Logger logger = rclcpp::get_logger("scenario_intersection");
  if (const auto ids {script_["TrafficLightId"]}) {
    for (const auto & each : ids) {
      ids_.emplace_back(each.as<std::size_t>());
    }
  } else {
    RCLCPP_ERROR_STREAM(
      logger, "Each element of node 'Intersection' requires hash 'TrafficLightId'.");
  }

  if (const auto controls {script_["Control"]}) {
    for (const auto & each : controls) {
      if (const auto & state_name {each["StateName"]}) {
        change_to_.emplace(state_name.as<std::string>(), Controller(each, logger));
      } else {
        RCLCPP_ERROR_STREAM(logger, "Each element of node 'Control' requires hash 'StateName'.");
      }
    }
  } else {
    RCLCPP_ERROR_STREAM(logger, "Each element of node 'Intersection' requires hash 'Control'.");
  }
}

bool Intersection::change_to(const std::string & the_state)
{
  // NOTE: Any unspecified state names are treated as "Blank" state
  return change_to_[current_state_ = the_state](*simulator_);
}

const std::vector<std::size_t> & Intersection::ids() const
{
  return ids_;
}

simulation_is Intersection::update()
{
  return simulation_is::ongoing;
}
}  // namespace scenario_intersection
