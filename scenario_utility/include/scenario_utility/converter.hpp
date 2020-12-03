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

#ifndef SCENARIO_UTILS_CONVERTER_H_INCLUDED
#define SCENARIO_UTILS_CONVERTER_H_INCLUDED

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2/transform_datatypes.h>

namespace scenario_utility
{
inline namespace converter
{

geometry_msgs::msg::Quaternion convert(geometry_msgs::msg::Vector3 rpy);
geometry_msgs::msg::Vector3 convert(geometry_msgs::msg::Quaternion quat);

}  // namespace converter
}  // namespace scenario_utility

#endif  // SCENARIO_UTILS_CONVERTER_H_INCLUDED
