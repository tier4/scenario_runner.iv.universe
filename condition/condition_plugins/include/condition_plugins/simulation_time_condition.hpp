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

#ifndef CONDITION_PLUGINS_SIMULATION_TIME_H_INCLUDED
#define CONDITION_PLUGINS_SIMULATION_TIME_H_INCLUDED

#include "rclcpp/rclcpp.hpp"
#include "scenario_conditions/condition_base.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_logger/logger.hpp"
#include "scenario_utility/scenario_utility.hpp"

namespace condition_plugins
{

class SimulationTimeCondition
  : public scenario_conditions::ConditionBase
{
  rclcpp::Duration duration_;

  std::string rule_;

  Comparator<rclcpp::Duration> compare_;
  rclcpp::Clock clock_;

public:
  SimulationTimeCondition();

  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> simulator) override;

  rclcpp::Duration elapsed() const noexcept;

  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
};

} // namespace condition_plugins

#endif // CONDITION_PLUGINS_SIMULATION_TIME_H_INCLUDED
