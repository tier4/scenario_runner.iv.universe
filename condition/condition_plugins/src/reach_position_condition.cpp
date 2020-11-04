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

#include "condition_plugins/reach_position_condition.hpp"

namespace condition_plugins
{

ReachPositionCondition::ReachPositionCondition()
  : scenario_conditions::ConditionBase { "ReachPosition" }
{}

bool ReachPositionCondition::configure(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> api_ptr)
try
{
  node_ = node;

  api_ptr_ = api_ptr;

  name_ = read_optional<std::string>(node_, "Name", name_);

  trigger_ = read_essential<std::string>(node_, "Trigger");

  const auto pose_stamped {read_essential<geometry_msgs::msg::PoseStamped>(node_, "Pose")};

  if (pose_stamped.header.frame_id == "/map") {
    target_pose_ = pose_stamped.pose;
  } else {
    target_pose_ =
      api_ptr_->getRelativePose(
      pose_stamped.header.frame_id, pose_stamped.pose);
  }

  tolerance_ = read_essential<float>(node_, "Tolerance");

  shift_ = read_optional<std::string>(node_, "Shift", "Center");

  keep_ = read_optional<bool>(node_, "Keep", false);

  return configured_ = true;
} catch (...) {
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

bool ReachPositionCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  if (keep_ and result_) {
    return result_;
  } else {
    if (!configured_) {
      SCENARIO_THROW_ERROR_ABOUT_INCOMPLETE_CONFIGURATION();
    }

    description_ =
      std::to_string(
        api_ptr_->getDistanceToArea(
          trigger_, target_pose_, shift_));

    if (api_ptr_->isObjectInArea(
        trigger_, target_pose_, tolerance_, boost::math::constants::two_pi<double>(), shift_))
    {
      SCENARIO_LOG_ABOUT_TOGGLE_CONDITION_RESULT();
      return result_ = true;
    } else {
      return result_ = false;
    }
  }
}

}  // namespace condition_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  condition_plugins::ReachPositionCondition,
  scenario_conditions::ConditionBase)
