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

#include <scenario_utility/converter.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace scenario_utility
{
namespace converter
{
geometry_msgs::msg::Quaternion convert(geometry_msgs::msg::Vector3 rpy)
{
  geometry_msgs::msg::Quaternion quat;
  tf2::Quaternion q;
  q.setRPY(rpy.x, rpy.y, rpy.z);
  quat.x = q.x();
  quat.y = q.y();
  quat.z = q.z();
  quat.w = q.w();
  return quat;
}

geometry_msgs::msg::Vector3 convert(geometry_msgs::msg::Quaternion quat)
{
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  return rpy;
}
}  // namespace converter
}  // namespace scenario_utility
