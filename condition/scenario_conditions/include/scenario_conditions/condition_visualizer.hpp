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

#ifndef SCENARIO_CONDITIONS_CONDITION_LOGGER_H_INCLUDED
#define SCENARIO_CONDITIONS_CONDITION_LOGGER_H_INCLUDED

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace scenario_conditions
{
// pre declaration
class ConditionManager;

class ConditionVisualizer
{
public:
  ConditionVisualizer(const rclcpp::Node::SharedPtr node);
  void publishMarker(ConditionManager& manager);

private:
  void addMarker(std::string name, bool result, bool is_success_condition);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
};
}  // namespace scenario_conditions
#endif  // SCENARIO_CONDITIONS_CONDITION_LOGGER_H_INCLUDED