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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "action_plugins/acceleration_action.hpp"

namespace action_plugins
{

void AccelerationAction::configure(
  const YAML::Node & node,
  const std::vector<std::string> actors,
  const std::shared_ptr<ScenarioAPI> & api_ptr)
try
{
  node_ = node;
  actors_ = actors;
  api_ptr_ = api_ptr;

  if (actors_.empty()) {
    SCENARIO_WARNING_ABOUT_NO_ACTORS_SPECIFIED();
  }

  name_ = read_optional<std::string>(node_, "Name", name_);

  call_with_essential(
    node_, "Params", [&](const auto & node) mutable
    {
      static_assert(
        std::numeric_limits<float>::has_signaling_NaN,
        "Used type must have the signaling NaN value.");

      min_ = read_optional<float>(node, "Min", std::numeric_limits<float>::signaling_NaN());
      max_ = read_optional<float>(node, "Max", std::numeric_limits<float>::signaling_NaN());

      if (0 < min_) {
        SCENARIO_WARN_STREAM(
          CATEGORY(),
          "'Min' value of Acceleration Action should be negative (You specified positive value " <<
            min_ << ")");
      }

      if (max_ < 0) {
        SCENARIO_WARN_STREAM(
          CATEGORY(),
          "'Max' value of Acceleration Action should be positive (You specified negative value " <<
            max_ << ")");
      }
    });
} catch (...) {
  SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION();
}

void AccelerationAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  for (const auto & actor : actors_) {
    if (!std::isnan(min_) && !(*api_ptr_).changeNPCAccelMin(actor, min_)) {
      SCENARIO_ERROR_THROW(
        CATEGORY(),
        type_ << "Action failed to change " << actor <<
          "'s minimum acceleration (Note: This action cannot be used for Ego Type entities).");
    }

    if (!std::isnan(max_) && !(*api_ptr_).changeNPCAccelMax(actor, max_)) {
      SCENARIO_ERROR_THROW(
        CATEGORY(),
        type_ << "Action failed to change " << actor <<
          "'s maximum acceleration (Note: This action cannot be used for Ego Type entities).");
    }
  }
}

}  // namespace action_plugins

PLUGINLIB_EXPORT_CLASS(action_plugins::AccelerationAction, scenario_actions::EntityActionBase)
