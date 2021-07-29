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

#include "scenario_conditions/condition_visualizer.hpp"
#include "scenario_conditions/condition_manager.hpp"


namespace scenario_conditions
{
ConditionVisualizer::ConditionVisualizer(const rclcpp::Node::SharedPtr node)
: node_(node)
{
  pub_marker_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "condition_marker",
    rclcpp::QoS{1});
}

void ConditionVisualizer::publishMarker(ConditionManager & manager)
{
  marker_array_.markers.clear();
  bool is_success_condition;
  auto add_func = [this, &is_success_condition](std::shared_ptr<ConditionBase> condition) {
      if (condition) {
        addMarker(condition->getName(), condition->getResult(), is_success_condition);
      }
    };
  is_success_condition = false;
  manager.applyVisitorForFailureConditions(add_func);
  is_success_condition = true;
  manager.applyVisitorForSuccessConditions(add_func);
  pub_marker_->publish(marker_array_);
}

void ConditionVisualizer::addMarker(std::string name, bool result, bool is_success_condition)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = node_->now();

  marker.ns = "conditions";
  marker.id = marker_array_.markers.size();

  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0);

  marker.scale.z = 2.0;

  marker.pose.position.x = 0;
  marker.pose.position.y = marker.scale.z * marker.id;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.color.r = 0.7f;
  marker.color.g = 0.7f;
  marker.color.b = 0.7f;
  marker.color.a = 1.0;

  std::string text;
  text += (is_success_condition ? "[S][All]" : "[F][Any]");
  if (result) {
    marker.color.g = 1.0f;
  } else {
    marker.color.r = 1.0f;
  }
  text += name;
  marker.text = text;
  marker_array_.markers.push_back(marker);
}
}  // namespace scenario_conditions
