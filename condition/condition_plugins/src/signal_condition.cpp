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

#include "condition_plugins/signal_condition.hpp"

namespace condition_plugins
{

size_t SignalCondition::occurrence { 0 };

SignalCondition::SignalCondition()
  : scenario_conditions::ConditionBase { "Signal", occurrence++ }
{}

bool SignalCondition::configure(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> simulator)
try
{
  node_ = node;

  simulator_ = simulator;

  name_ = read_optional<std::string>(node_, "Name", name_);

  trigger_ = read_essential<std::string>(node_, "Trigger");

  state_ = read_essential<std::string>(node_, "State");

  keep_ = read_optional<bool>(node_, "Keep", false);

  return configured_ = true;
} catch (...) {
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

bool SignalCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> & intersections)
{
  // ROS_WARN_STREAM("IDs = [");
  //
  // for (const auto& id : (*intersections).at(trigger_).ids())
  // {
  //   ROS_WARN_STREAM("  " << id);
  // }
  //
  // ROS_WARN_STREAM("]");

  if (keep_ && result_) {
    return result_;
  }
  else
  {
    description_ = (*intersections).at(trigger_).current_state();
    return result_ = (*intersections).at(trigger_).is(state_);
  }
}

} // namespace condition_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(condition_plugins::SignalCondition, scenario_conditions::ConditionBase)
