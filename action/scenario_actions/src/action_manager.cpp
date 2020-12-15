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

#include "scenario_actions/action_manager.hpp"

#include "scenario_logger/logger.hpp"
#include "scenario_utility/parse.hpp"

namespace scenario_actions
{

ActionManager::ActionManager(
  const YAML::Node & actions_node,
  const std::vector<std::string> & actors,
  const std::shared_ptr<ScenarioAPI> & api_ptr)
try
: actions_node_{actions_node},
  actors_{actors},
  api_ptr_{api_ptr}
{
  for (const auto & action_node : actions_node_) {
    loadPlugin(action_node);
  }
} catch (...) {
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize actions.");
}

void ActionManager::loadPlugin(const YAML::Node & node)
try
{
  const auto type {read_essential<std::string>(node, "Type") + "Action"};

  pluginlib::ClassLoader<scenario_actions::EntityActionBase> loader("scenario_actions",
    "scenario_actions::EntityActionBase");

  const std::vector<std::string> classes = loader.getDeclaredClasses();

  auto iter =
    std::find_if(
    classes.begin(), classes.end(),
    [&](std::string c)
    {
      return loader.getName(c) == type;
    });

  if (iter == classes.end()) {
    SCENARIO_ERROR_THROW(CATEGORY(), "There is no plugin of type '" << type << "'.");
  } else {
    std::shared_ptr<EntityActionBase> plugin = loader.createSharedInstance(*iter);
    plugin->configure(node, actors_, api_ptr_);
    actions_.push_back(plugin);
  }
} catch (...) {
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to load action plugin.");
}

void ActionManager::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager> & intersection_manager)
try
{
  for (const auto & each : actions_) {
    each->run(intersection_manager);
  }
} catch (...) {
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to execute action.");
}

} // namespace scenario_actions
