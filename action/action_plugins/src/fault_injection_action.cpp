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

#include <memory>
#include <string>
#include <vector>

#include "action_plugins/fault_injection_action.hpp"

namespace action_plugins
{

void FaultInjectionAction::configure(
  const YAML::Node & node,
  const std::vector<std::string> actors,
  const std::shared_ptr<ScenarioAPI> & api_ptr)
try
{
  node_ = node;
  actors_ = actors;
  api_ptr_ = api_ptr;

  if (actors_.empty()) {
    SCENARIO_WARNING_ABOUT_NO_ACTORS_SPECIFIED();
  }

  name_ = read_optional<std::string>(node_, "Name", name_);

  call_with_essential(
    node_, "Params", [&](const auto & node) mutable
    {
      target_node_ = read_essential<std::string>(node, "Node");
    });
} catch (...) {
  SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION();
}

void FaultInjectionAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  // TODO(yunus.caliskan): Replace the logic for ros2 or disable this action altogether.
  const auto command {"rosnode kill " + target_node_};

  if (::system(command.c_str())) {
    SCENARIO_ERROR_THROW(
      CATEGORY(),
      type_ << "Action failed to execute command \"" << command << "\".");
  } else {
    SCENARIO_INFO_STREAM(
      CATEGORY(),
      type_ << "Action killed ROS node \'" << target_node_ << "\'.");
  }
}

}  // namespace action_plugins

PLUGINLIB_EXPORT_CLASS(action_plugins::FaultInjectionAction, scenario_actions::EntityActionBase)
