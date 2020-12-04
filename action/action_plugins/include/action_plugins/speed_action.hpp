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

#ifndef INCLUDED_ACTION_PLUGINS_SPEED_ACTION_H
#define INCLUDED_ACTION_PLUGINS_SPEED_ACTION_H

#include <pluginlib/class_list_macros.hpp>

#include <scenario_actions/entity_action_base.hpp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_logger/logger.hpp>
#include <scenario_utility/scenario_utility.hpp>

namespace action_plugins
{

class SpeedAction // AbsoluteSpeedAction
  : public scenario_actions::EntityActionBase
{
  float value_;

public:
  SpeedAction();

  void configure(
    const YAML::Node&,
    const std::vector<std::string>,
    const std::shared_ptr<ScenarioAPI>&) override;

  auto run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    -> void override;
};

} // namespace action_plugins

#endif // INCLUDED_ACTION_PLUGINS_SPEED_ACTION_H

