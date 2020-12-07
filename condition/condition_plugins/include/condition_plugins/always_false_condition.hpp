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

#ifndef TEST_CONDITIONS_ALWAYS_FALSE_CONDITION_H_INCLUDED
#define TEST_CONDITIONS_ALWAYS_FALSE_CONDITION_H_INCLUDED

#include "scenario_conditions/condition_base.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_utility/scenario_utility.hpp"

namespace condition_plugins
{

class AlwaysFalseCondition
  : public scenario_conditions::ConditionBase
{
public:
  AlwaysFalseCondition();

  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;

  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) override;
};

} // namespace condition_plugins

#endif // TEST_CONDITIONS_ALWAYS_FALSE_CONDITION_H_INCLUDED

