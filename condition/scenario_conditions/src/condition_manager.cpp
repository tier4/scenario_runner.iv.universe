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

#include "scenario_conditions/condition_manager.hpp"

namespace scenario_conditions
{
ConditionManager::ConditionManager(
  YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr,
  rclcpp::Node::SharedPtr node_ptr)
: visualizer(node_ptr)
{
  try {
    call_with_optional(
      node, "Success", [&](const auto & node) {
        call_with_essential(
          node, "All", [&](const auto & node) {
            for (const auto & each : node) {
              success_conditions_.push_back(loadPlugin(each, api_ptr));
            }
          });
      });

    call_with_optional(
      node, "Failure", [&](const auto & node) {
        call_with_essential(
          node, "Any", [&](const auto & node) {
            for (const auto & each : node) {
              failure_conditions_.push_back(loadPlugin(each, api_ptr));
            }
          });
      });
  } catch (...) {
    SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize conditions.");
  }
}

ConditionManager::condition_type ConditionManager::loadPlugin(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> api_ptr)
{
  try {
    const auto type{read_essential<std::string>(node, "Type") + "Condition"};

    pluginlib::ClassLoader<scenario_conditions::ConditionBase> loader(
      "scenario_conditions", "scenario_conditions::ConditionBase");

    std::vector<std::string> classes = loader.getDeclaredClasses();

    auto iter = std::find_if(
      classes.begin(), classes.end(), [&](std::string c) {return loader.getName(c) == type;});

    if (iter == classes.end()) {
      SCENARIO_ERROR_THROW(CATEGORY(), "There is no plugin of type '" << type << "'.");
    } else {
      auto plugin = loader.createSharedInstance(*iter);
      plugin->configure(node, api_ptr);
      return plugin;
    }
  } catch (...) {
    SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to load condition plugin.");
  }
}

simulation_is ConditionManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> & intersection_manager)
{
  visualizer.publishMarker(*this);

  if (std::accumulate(
      failure_conditions_.begin(), failure_conditions_.end(),
      false,
      [&](const auto & lhs, const auto & rhs)
      {
        const auto result {rhs ? (*rhs).update(intersection_manager) : false};
        return lhs or result;
      }))
  {
    return simulation_is::failed;
  }

  if (std::accumulate(
      success_conditions_.begin(), success_conditions_.end(),
      true,
      [&](const auto & lhs, const auto & rhs)
      {
        const auto result {rhs ? (*rhs).update(intersection_manager) : false};
        return lhs and result;
      }))
  {
    return simulation_is::succeeded;
  }

  return simulation_is::ongoing;
}

void ConditionManager::applyVisitorForSuccessConditions(
  const std::function<void(std::shared_ptr<ConditionBase>)> & visitor)
{
  for (const auto & condition : success_conditions_) {
    visitor(condition);
  }
}

void ConditionManager::applyVisitorForFailureConditions(
  const std::function<void(std::shared_ptr<ConditionBase>)> & visitor)
{
  for (const auto & condition : failure_conditions_) {
    visitor(condition);
  }
}

}
