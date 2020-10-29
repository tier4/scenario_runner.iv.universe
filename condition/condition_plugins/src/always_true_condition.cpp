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

#include "condition_plugins/always_true_condition.hpp"

namespace condition_plugins
{

std::size_t AlwaysTrueCondition::occurrence { 0 };

AlwaysTrueCondition::AlwaysTrueCondition()
: scenario_conditions::ConditionBase{"AlwaysTrue", occurrence++}
{
  result_ = true;
}

bool AlwaysTrueCondition::configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr)
try
{
  node_ = node;
  api_ptr_ = api_ptr;
  name_ = read_optional<std::string>(node_, "Name", name_);

  return configured_ = true;
} catch (...) {
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

bool AlwaysTrueCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  return result_;
}

} // namespace condition_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(condition_plugins::AlwaysTrueCondition, scenario_conditions::ConditionBase)
