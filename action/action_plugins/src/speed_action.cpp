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

#include "action_plugins/speed_action.hpp"

namespace action_plugins
{

SpeedAction::SpeedAction()
  : EntityActionBase {"Speed"}
{}

void SpeedAction::configure(
  const YAML::Node& node,
  const std::vector<std::string> actors,
  const std::shared_ptr<ScenarioAPI>& simulator)
try
{
  node_ = node;
  actors_ = actors;
  api_ptr_ = simulator;

  if (actors_.empty())
  {
    SCENARIO_WARNING_ABOUT_NO_ACTORS_SPECIFIED();
  }

  name_ = read_optional<std::string>(node_, "Name", name_);

  call_with_essential(node_, "Params", [&](const auto& node) mutable
  {
    value_ = read_optional<float>(node, "Value", 0.0);
  });
}
catch (...)
{
  SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION();
}

void SpeedAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  for (const auto& actor : actors_)
  {
    if (not api_ptr_->isEgoCarName(actor))
    {
      if (not (*api_ptr_).changeNPCVelocity(actor, value_))
      {
        SCENARIO_ERROR_THROW(CATEGORY(),
          type_ << "Action failed to change " << actor << "'s speed.");
      }
    }
    else
    {
      if (not api_ptr_->setMaxSpeed(value_))
      {
        SCENARIO_ERROR_THROW(CATEGORY(),
          type_ << "Action failed to change " << actor << "'s max-speed (Note: This action implicitly acts as a maximum speed setting for Type: Ego entities.");
      }
    }
  }
}


} // namespace action_plugins

PLUGINLIB_EXPORT_CLASS(action_plugins::SpeedAction, scenario_actions::EntityActionBase)

