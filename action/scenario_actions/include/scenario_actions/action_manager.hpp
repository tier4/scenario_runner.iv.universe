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

#ifndef SCENARIO_ACTIONS_ACTION_MANAGER_H_INCLUDED
#define SCENARIO_ACTIONS_ACTION_MANAGER_H_INCLUDED

#include <scenario_actions/entity_action_base.hpp>
#include <scenario_api/scenario_api_core.hpp>
#include <scenario_intersection/intersection_manager.hpp>

#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <vector>


namespace scenario_actions
{

class ActionManager
{
public:
  ActionManager(
    const YAML::Node& node,
    const std::vector<std::string>& actors,
    const std::shared_ptr<ScenarioAPI>& api_ptr);

  auto run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    -> void;

private:
  const YAML::Node actions_node_;
  const std::vector<std::string> actors_;

  const std::shared_ptr<ScenarioAPI> api_ptr_;

  std::vector<std::shared_ptr<EntityActionBase>> actions_;

  void loadPlugin(const YAML::Node& node);
};

}  // namespace scenario_actions

#endif // SCENARIO_ACTIONS_ACTION_MANAGER_H_INCLUDED
