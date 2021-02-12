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

#ifndef ACTION_PLUGINS__LANE_CHANGE_ACTION_HPP_
#define ACTION_PLUGINS__LANE_CHANGE_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "scenario_actions/entity_action_base.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_utility/scenario_utility.hpp"

namespace action_plugins
{

class LaneChangeAction
  : public scenario_actions::EntityActionBase
{
  int target_lanelet_;

public:
  LaneChangeAction();

  void configure(
    const YAML::Node & node,
    const std::vector<std::string> actors,
    const std::shared_ptr<ScenarioAPI> & api_ptr) override;

  auto run(
    const std::shared_ptr<scenario_intersection::IntersectionManager> &)
  ->void override;
};

}  // namespace action_plugins

#endif  // ACTION_PLUGINS__LANE_CHANGE_ACTION_HPP_
