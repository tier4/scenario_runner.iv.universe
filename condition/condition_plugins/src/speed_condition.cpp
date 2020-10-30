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

#include "condition_plugins/speed_condition.hpp"

namespace condition_plugins
{

SpeedCondition::SpeedCondition()
  : scenario_conditions::ConditionBase { "Speed" }
{}

bool SpeedCondition::configure(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> api_ptr)
try
{
  node_ = node;
  api_ptr_ = api_ptr;

  name_ = read_optional<std::string>(node_, "Name", name_);

  trigger_ = read_essential<std::string>(node_, "Trigger");

  target_value_ = read_essential<float>(node_, "Value");

  if (!parseRule<float>(read_essential<std::string>(node_, "Rule"), compare_)) {
    return configured_ = false;
  }

  keep_ = read_optional<bool>(node_, "Keep", false);

  return configured_ = true;
} catch (...) {
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

bool SpeedCondition::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  if (keep_ && result_) {
    return true;
  }
  else
  {
    if ((*api_ptr_).isEgoCarName(trigger_))
    {
      return result_ = compare_(value_ = (*api_ptr_).getVelocity(), target_value_);
    }
    else
    {
      if (!(*api_ptr_).getNPCVelocity(trigger_, &value_))
      {
        RCLCPP_ERROR_STREAM(
          api_ptr_->get_logger().get_child("AccelerationCondition"),
          "Invalid trigger name specified for " << getType() << " condition named " << getName());
        return result_ = false;
      }
      else
      {
        return result_ = compare_(value_, target_value_);
      }
    }
  }
}

}  // namespace condition_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(condition_plugins::SpeedCondition, scenario_conditions::ConditionBase)
