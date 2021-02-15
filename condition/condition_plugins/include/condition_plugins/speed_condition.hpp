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

#ifndef CONDITION_PLUGINS_SPEED_CONDITION_H_INCLUDED
#define CONDITION_PLUGINS_SPEED_CONDITION_H_INCLUDED

#include "scenario_conditions/condition_base.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_logger/logger.hpp"
#include "scenario_utility/scenario_utility.hpp"
#include <sstream>

namespace condition_plugins
{

class SpeedCondition : public scenario_conditions::ConditionBase
{
public:
  SpeedCondition();
  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) override;

private:
  std::string rule_;
  std::string trigger_;
  float target_value_;
  Comparator<float> compare_;
};

}  // namespace condition_plugins

#endif  // CONDITION_PLUGINS_SPEED_CONDITION_H_INCLUDED
