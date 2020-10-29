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

#ifndef CONDITION_PLUGINS_COLLISION_BY_ENTITY_CONDITION_H_INCLUDED
#define CONDITION_PLUGINS_COLLISION_BY_ENTITY_CONDITION_H_INCLUDED

#include "scenario_conditions/condition_base.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_utility/scenario_utility.hpp"

namespace condition_plugins
{
class CollisionByEntityCondition : public scenario_conditions::ConditionBase
{
  static std::size_t occurrence;

public:
  CollisionByEntityCondition();
  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) override;

private:
  std::string trigger_, target_entity_;
};
}  // namespace condition_plugins

#endif  // CONDITION_PLUGINS_COLLISION_BY_ENTITY_CONDITION_H_INCLUDED
