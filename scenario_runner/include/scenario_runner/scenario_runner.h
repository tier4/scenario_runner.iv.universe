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

#ifndef SCENARIO_RUNNER_SCENARIO_RUNNER_H_INCLUDED
#define SCENARIO_RUNNER_SCENARIO_RUNNER_H_INCLUDED

#include "scenario_expression/expression.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_logger/logger.hpp"
#include "scenario_sequence/sequence_manager.hpp"
#include "scenario_utility/scenario_utility.hpp"
#include "scenario_entities/entity_manager.hpp"

#include <memory>
#include "autoware_debug_msgs/msg/string_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace scenario_runner
{
class ScenarioRunner : public rclcpp::Node
{
public:
  ScenarioRunner();
  void run();

  double current_mileage() const
  {
    return simulator_->getMoveDistance();
  }

  void spin_simulator();

  simulation_is currently;

private:
  rclcpp::TimerBase::SharedPtr timer_;

  const std::shared_ptr<ScenarioAPI> simulator_;

  rclcpp::Publisher<autoware_debug_msgs::msg::StringStamped>::SharedPtr publisher_;

  std::string scenario_path_;

  YAML::Node scenario_;

  scenario_expression::Context context;

  scenario_expression::Expression success, failure;

  std::shared_ptr<scenario_entities::EntityManager> entity_manager_;
  std::shared_ptr<scenario_sequence::SequenceManager> sequence_manager_;
  std::shared_ptr<scenario_intersection::IntersectionManager> intersection_manager_;

  void update();
};

}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER_SCENARIO_RUNNER_H_INCLUDED
