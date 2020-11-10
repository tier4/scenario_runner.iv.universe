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

#include "scenario_entities/entity_base.hpp"
#include "scenario_logger/simple_logger.hpp"

#include <sstream>

namespace scenario_entities
{

bool EntityBase::configure(
  const YAML::Node & entity,
  const std::shared_ptr<ScenarioAPI> & api)
try
{
  std::stringstream ss;
  ss << type_ << "Entity-" << static_cast<const void *>(this);
  name_ =
    read_optional<std::string>(
    entity, "Name", ss.str());

  return static_cast<bool>(api_ = api);
} catch (...) {
  SCENARIO_ERROR_RETHROW(
    CATEGORY(),
    "Failed to configure entity named '" << name_ << "' of type " << type_ << ".");
}

bool EntityBase::setStory(const YAML::Node & story)
try
{
  if (story["Init"]) {
    init_ = story["Init"];
  }

  if (story["Act"]) {
    act_ = story["Act"];
  }

  if (story["EndCondition"]) {
    end_condition_ = story["EndCondition"];
  }

  call_with_essential(
    init_, "Entity", [&](const auto & node) mutable
    {
      for (const YAML::Node & each : node) {
        if (read_essential<std::string>(each, "Name") == name_) {
          return init_entity_ = each;
        }
      }

      SCENARIO_ERROR_THROW(
        CATEGORY(),
        "There is no initialization for entity named '" << name_ << "' of type " << type_ << ".");
    });

  return init_entity_;
} catch (...) {
  SCENARIO_ERROR_RETHROW(
    CATEGORY(),
    "Failed to read story to entity named '" << name_ << "' of type " << type_ << ".");
}

bool EntityBase::init()
try
{
  LOG_SIMPLE(info() << "Parse 'Story.Init.Entity[" << name_ << "].InitialStates");
  call_with_essential(init_entity_, "InitialStates", [&](const auto& node) mutable
  {
    const auto type { type_ != "Vehicle" ? boost::to_lower_copy(type_) : "car" };

    api_->addNPC(
      type,
      name_,
      read_essential<geometry_msgs::Pose>(node, "Pose"),
      read_optional<float>(node, "Speed", 0),
      false,
      read_optional<std::string>(node, "Shift", "Center"));
  });

  LOG_SIMPLE(info() << "Parse 'Story.Init.Entity[" << name_ << "].Actions");
  call_with_optional(init_entity_, "Actions", [&](const auto& node) mutable
  {
    action_manager_ =
      std::make_shared<scenario_actions::ActionManager>(
        node, std::vector<std::string> {name_}, api_);

      action_manager_->run(nullptr);
    });

  return true;
} catch (...) {
  SCENARIO_ERROR_RETHROW(
    CATEGORY(),
    "Failed to initialize entity named '" << name_ << "' of type " << type_ << ".");
}

[[deprecated]]
simulation_is EntityBase::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> & intersection_manager)
{
  return simulation_is::succeeded;
}

} // namespace scenario_entities
