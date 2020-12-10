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

#include "scenario_utility/parse.hpp"

#include <string>
#include <vector>

inline namespace scenario_utility
{
inline namespace parse
{
#define DEFINE_READ_ESSENTIAL_SPECIALIZATION(TYPENAME) \
  READ_ESSENTIAL_SPECIALIZED_SIGNATURE(TYPENAME) \
  { \
    return call_with_essential(node, key, read_as<TYPENAME>); \
  } \
  static_assert(true, "semicolon required after this macro")

template<>
geometry_msgs::msg::Point read_as<geometry_msgs::msg::Point>(const YAML::Node & node)
{
  geometry_msgs::msg::Point point {};

  point.x = read_essential<float>(node, "X");
  point.y = read_essential<float>(node, "Y");
  point.z = read_essential<float>(node, "Z");

  return point;
}

DEFINE_READ_ESSENTIAL_SPECIALIZATION(geometry_msgs::msg::Point);

template<>
geometry_msgs::msg::Quaternion read_as<geometry_msgs::msg::Quaternion>(const YAML::Node & node)
{
  geometry_msgs::msg::Quaternion quaternion {};

  quaternion.x = read_essential<float>(node, "X");
  quaternion.y = read_essential<float>(node, "Y");
  quaternion.z = read_essential<float>(node, "Z");
  quaternion.w = read_essential<float>(node, "W");

  return quaternion;
}

DEFINE_READ_ESSENTIAL_SPECIALIZATION(geometry_msgs::msg::Quaternion);

template<>
geometry_msgs::msg::Pose read_as<geometry_msgs::msg::Pose>(const YAML::Node & node)
{
  geometry_msgs::msg::Pose pose {};

  pose.position = read_essential<geometry_msgs::msg::Point>(node, "Position");
  pose.orientation = read_essential<geometry_msgs::msg::Quaternion>(node, "Orientation");

  return pose;
}

DEFINE_READ_ESSENTIAL_SPECIALIZATION(geometry_msgs::msg::Pose);

template<>
geometry_msgs::msg::PoseStamped read_as<geometry_msgs::msg::PoseStamped>(const YAML::Node & node)
{
  geometry_msgs::msg::PoseStamped pose_stamped {};

  pose_stamped.header.frame_id = read_optional<std::string>(node, "FrameId", "/map");
  // pose_stamped.header.stamp = ros::Time::now();
  // pose_stamped.pose = read_essential<geometry_msgs::msg::Pose>(node, "Pose");
  pose_stamped.pose.position = read_essential<geometry_msgs::msg::Point>(node, "Position");
  pose_stamped.pose.orientation =
    read_essential<geometry_msgs::msg::Quaternion>(node, "Orientation");

  return pose_stamped;
}

DEFINE_READ_ESSENTIAL_SPECIALIZATION(geometry_msgs::msg::PoseStamped);

std::vector<std::string> split(std::string s)
{
  std::vector<std::string> v;
  std::stringstream ss{s};
  std::string buf;
  while (std::getline(ss, buf, ',')) {
    v.push_back(buf);
  }
  return v;
}
}  // namespace parse
}  // namespace scenario_utility
