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

#include <chrono>
#include <thread>

#include "scenario_logger/logger.hpp"
#include "scenario_runner/scenario_runner.h"

namespace scenario_runner
{
ScenarioRunner::ScenarioRunner()
: Node("ScenarioRunnerNode"),
  currently{simulation_is::ongoing},
  simulator_{std::make_shared<ScenarioAPI>()},
  scenario_path_{declare_parameter("scenario_path").get<std::string>()}
{
  simulator_->init();
  if (not (*simulator_).waitAutowareInitialize()) {
    SCENARIO_ERROR_THROW(CATEGORY(), "Failed to initialize Autoware.");
  }

  try {
    scenario_ = YAML::LoadFile(scenario_path_);
  } catch (...) {
    SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to load YAML file \"" << scenario_path_ << "\".");
  }
}

void ScenarioRunner::spin_simulator()
{
  rclcpp::spin_some(simulator_);
}

void ScenarioRunner::run()
try
{
  context.define(simulator_);

  call_with_essential(
    scenario_, "Entity", [&](const auto & node) mutable
    {
      context.define(
        entity_manager_ =
        std::make_shared<scenario_entities::EntityManager>(
          node, simulator_));
    });

  call_with_optional(
    scenario_, "Intersection", [&](const auto & node) mutable
    {
      context.define(
        intersection_manager_ =
        std::make_shared<scenario_intersection::IntersectionManager>(
          node, simulator_));
    });

  call_with_essential(
    scenario_, "Story", [&](const auto & node) mutable
    {
      context.entities().setStory(node);
      context.entities().initialize();

      call_with_essential(
        node, "Init", [&](const auto & node) mutable
        {
          call_with_optional(
            node, "Intersection", [&](const auto & node) mutable
            {
              context.intersections().initialize(node);
            });
        });

      // call_with_essential(node, "Act", [&](const auto& node) mutable
      // {
      //   sequence_manager_ =
      //     std::make_shared<scenario_sequence::SequenceManager>(
      //       context, node);
      // });

      sequence_manager_ =
      std::make_shared<scenario_sequence::SequenceManager>(
        context, node["Act"]);

      call_with_essential(
        node, "EndCondition", [&](const auto & node) mutable
        {
          call_with_optional(
            node, "Success", [&](const auto & node) mutable
            {
              success = scenario_expression::read(context, node);
              SCENARIO_INFO_STREAM(CATEGORY(), "Loaded success condition: " << success);
            });

          call_with_optional(
            node, "Failure", [&](const auto & node) mutable
            {
              failure = scenario_expression::read(context, node);
              SCENARIO_INFO_STREAM(CATEGORY(), "Loaded failure condition: " << failure);
            });
        });
    });

  SCENARIO_INFO_STREAM(CATEGORY(), "Waiting for the simulator API to be ready.");
  simulator_->waitAPIReady();
  SCENARIO_INFO_STREAM(CATEGORY(), "Simulator API is ready.");

  auto timer_callback = std::bind(&ScenarioRunner::update, this);
  const auto period = std::chrono::milliseconds{10UL};
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  if (not simulator_->sendEngage(true)) {
    SCENARIO_ERROR_THROW(CATEGORY(), "Failed to send engage.");
  }
  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "ScenarioRunner engaged Autoware.");

  scenario_logger::log.initialize(this->now()); // NOTE: initialize logger's clock here.
  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "Simulation started.");
} catch (...) {
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize ScenarioRunner.");
}

void ScenarioRunner::update() try
{
  std::cout << (indent++) << "ScenarioRunnerContext: {\n";

  scenario_logger::log.updateMoveDistance(simulator_->getMoveDistance());

  (*sequence_manager_).update(intersection_manager_);

  // currently = (*entity_manager_).update(intersection_manager_);

  if (failure.evaluate(context)) {
    currently = simulation_is::failed;
  } else if (success.evaluate(context)) {
    currently = simulation_is::succeeded;
  } else {
    currently = simulation_is::ongoing;
  }

  std::cout << (--indent) << "}" << std::endl;
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to update simulation.");
}

}  // namespace scenario_runner
