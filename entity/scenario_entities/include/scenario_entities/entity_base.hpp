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

#ifndef SCENARIO_ENTITIES_ENTITIY_BASE_H_INCLUDED
#define SCENARIO_ENTITIES_ENTITIY_BASE_H_INCLUDED

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "scenario_actions/action_manager.hpp"
#include "scenario_api/scenario_api_core.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_logger/logger.hpp"
#include "scenario_utility/scenario_utility.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

namespace scenario_entities
{

class EntityBase
{
public:
  EntityBase(const std::string & type)
  : type_{type}
  {}

  const std::string & getName() const
  {
    return name_;
  }

  const std::string & getType() const
  {
    return type_;
  }

  virtual bool configure(
    const YAML::Node & entity,
    const std::shared_ptr<ScenarioAPI> & api);

  virtual bool setStory(const YAML::Node & story);

  virtual bool init();

  virtual simulation_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager> &);

protected:
  std::string name_;
  std::string type_;

  YAML::Node act_;
  YAML::Node end_condition_;
  YAML::Node init_;
  YAML::Node init_entity_;

  std::shared_ptr<ScenarioAPI> api_;

  std::shared_ptr<scenario_actions::ActionManager> action_manager_;

  geometry_msgs::msg::PoseStamped pose_stamped_;

  float speed_;
};
}  // namespace scenario_entities
#endif  // SCENARIO_ENTITIES_ENTITIY_BASE_H_INCLUDED
