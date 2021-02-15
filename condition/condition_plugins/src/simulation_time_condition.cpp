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

#include "condition_plugins/simulation_time_condition.hpp"

namespace condition_plugins
{

SimulationTimeCondition::SimulationTimeCondition()
  : scenario_conditions::ConditionBase { "SimulationTime" }, duration_(0, 0)
{}

bool SimulationTimeCondition::configure(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> simulator)
try
{
  node_ = node;

  name_ = read_optional<std::string>(node_, "Name", name_);

  duration_ =
    rclcpp::Duration(
    read_essential<float>(node_, "Value"), 0);

  if (!parseRule<rclcpp::Duration>(
      read_essential<std::string>(node_, "Rule"),
      compare_))
  {
    return configured_ = false;
  }

  keep_ = read_optional<bool>(node_, "Keep", false);

  return configured_ = true;
} catch (...) {
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

rclcpp::Duration SimulationTimeCondition::elapsed() const noexcept
{
  const auto & begin = scenario_logger::log.begin();
  return rclcpp::Clock{begin.get_clock_type()}.now() - begin;
}

bool SimulationTimeCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  if (keep_ && result_) {
    return true;
  }
  else
  {
    description_ = std::to_string(elapsed().seconds());
    return result_ = compare_(elapsed(), duration_);
  }
}

} // namespace condition_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  condition_plugins::SimulationTimeCondition,
  scenario_conditions::ConditionBase)
