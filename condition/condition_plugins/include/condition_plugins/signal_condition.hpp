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

#ifndef INCLUDED_CONDITION_PLUGINS_SIGNAL_CONDITION_H
#define INCLUDED_CONDITION_PLUGINS_SIGNAL_CONDITION_H

#include "scenario_conditions/condition_base.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_logger/logger.hpp"
#include "scenario_utility/scenario_utility.hpp"

namespace condition_plugins
{

class SignalCondition
  : public scenario_conditions::ConditionBase
{
  std::shared_ptr<ScenarioAPI> simulator_;

  std::string trigger_, state_;

public:
  SignalCondition();

  bool configure(YAML::Node, std::shared_ptr<ScenarioAPI>) override;

  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
};

} // namespace condition_plugins

#endif // INCLUDED_CONDITION_PLUGINS_SIGNAL_CONDITION_H

