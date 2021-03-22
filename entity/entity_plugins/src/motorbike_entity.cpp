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

#include "entity_plugins/motorbike_entity.hpp"

namespace entity_plugins
{

MotorBikeEntity::MotorBikeEntity()
: scenario_entities::EntityBase{"MotorBike"}
{}

}  // namespace entity_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(entity_plugins::MotorBikeEntity, scenario_entities::EntityBase)
