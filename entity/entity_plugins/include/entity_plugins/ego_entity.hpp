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

#ifndef ENTITY_PLUGINS__EGO_ENTITY_HPP_
#define ENTITY_PLUGINS__EGO_ENTITY_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "scenario_entities/entity_base.hpp"

namespace entity_plugins
{

class EgoEntity
  : public scenario_entities::EntityBase
{
public:
  EgoEntity();

  bool init() override;

  bool configure(
    const YAML::Node &,
    const std::shared_ptr<ScenarioAPI> &)
  override;

private:
  std::string urdf_;
  std::string initial_frame_id_;

  std::shared_ptr<scenario_actions::ActionManager> action_manager_ptr_;
  std::shared_ptr<scenario_actions::ActionManager> initial_action_manager_ptr_;
};

}  // namespace entity_plugins

#endif  // ENTITY_PLUGINS__EGO_ENTITY_HPP_
