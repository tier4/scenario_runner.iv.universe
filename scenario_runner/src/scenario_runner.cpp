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

#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"

#include "scenario_logger/logger.hpp"
#include "scenario_logger/simple_logger.hpp"
#include "scenario_runner/scenario_runner.h"

namespace scenario_runner
{
ScenarioRunner::ScenarioRunner()
: Node("ScenarioRunnerNode"),
  currently{simulation_is::ongoing},
  publisher_{create_publisher<scenario_runner_msgs::msg::StringStamped>("context", rclcpp::QoS(1).transient_local())},
  simulator_{std::make_shared<ScenarioAPI>()},
  scenario_path_{declare_parameter("scenario_path").get<std::string>()}
{
  simulator_->init();
  if (not (*simulator_).waitAutowareInitialize())
  {
    SCENARIO_ERROR_THROW(CATEGORY(), "Failed to initialize Autoware.");
  }

  try
  {
    LOG_SIMPLE(info() << "Load scenario " << scenario_path_);
    scenario_ = YAML::LoadFile(scenario_path_);
  }
  catch (...)
  {
    LOG_SIMPLE(error() << "Failed to load scenario: " << scenario_path_);
    SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to load YAML file \"" << scenario_path_ << "\".");
  }
}

void ScenarioRunner::spin_simulator()
{
  rclcpp::spin_some(simulator_);
}

void ScenarioRunner::run() try
{
  LOG_SIMPLE(info() << "Parse scenario");
  context.define(simulator_);

  LOG_SIMPLE(info() << "Parse 'Entity'");
  call_with_essential(scenario_, "Entity", [&](const auto & node)
  {
    context.define(
      entity_manager_ =
        std::make_shared<scenario_entities::EntityManager>(
          node, simulator_));
    });

  LOG_SIMPLE(info() << "Parse 'Intersections'");
  call_with_optional(
    scenario_, "Intersection", [&](const auto & node)
    {
      context.define(
        intersection_manager_ =
        std::make_shared<scenario_intersection::IntersectionManager>(
          node, simulator_));
    });

  LOG_SIMPLE(info() << "Parse 'Story'");
  call_with_essential(
    scenario_, "Story", [&](const auto & node)
    {
      LOG_SIMPLE(info() << "Set 'Story' to entities");
      context.entities().setStory(node);

      LOG_SIMPLE(info() << "Initializing entities");
      context.entities().initialize();

      LOG_SIMPLE(info() << "Parse 'Story.Init'");
      call_with_essential(
        node, "Init", [&](const auto & node)
        {
          LOG_SIMPLE(info() << "Parse 'Story.Init.Intersection'");
          call_with_optional(
            node, "Intersection", [&](const auto & node)
            {
              context.intersections().initialize(node);
            });
        });

      LOG_SIMPLE(info() << "Parse 'Story.Act'");
      call_with_optional(node, "Act", [&](auto&& node)
      {
        sequence_manager_ =
          std::make_shared<scenario_sequence::SequenceManager>(
            context, node);
      });

      LOG_SIMPLE(info() << "Parse 'Story.EndCondition'");
      call_with_essential(
        node, "EndCondition", [&](const auto & node)
        {
          LOG_SIMPLE(info() << "Parse 'Story.EndCondition.Success'");
          call_with_optional(
            node, "Success", [&](const auto & node)
            {
              success = scenario_expression::read(context, node);
            });

          LOG_SIMPLE(info() << "Parse 'Story.EndCondition.Failure'");
          call_with_optional(
            node, "Failure", [&](const auto & node)
            {
              failure = scenario_expression::read(context, node);
            });
        });
    });

  LOG_SIMPLE(info() << "Parse completed. There is no syntax-error");

  LOG_SIMPLE(info() << "Waiting for simulation APIs to be ready");
  simulator_->waitAPIReady();
  LOG_SIMPLE(info() << "Simulation APIs are ready");

  auto timer_callback = std::bind(&ScenarioRunner::update, this);
  const auto period = std::chrono::milliseconds{50UL};
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  LOG_SIMPLE(info() << "Engage!");
  simulator_->sendEngage(true);

  LOG_SIMPLE(info() << "Initialize SimulationTime");
  scenario_logger::log.initialize(this->now()); // NOTE: initialize logger's clock here.

  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "Simulation started.");
} catch (...) {
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize ScenarioRunner.");
}

void ScenarioRunner::update() try
{
  scenario_logger::log.updateMoveDistance(simulator_->getMoveDistance());

  (*sequence_manager_).update(intersection_manager_);

  const auto fulfilled_failure_condition { failure.evaluate(context) };
  const auto fulfilled_success_condition { success.evaluate(context) };

  if (fulfilled_failure_condition)
  {
    currently = simulation_is::failed;
  }
  else if (fulfilled_success_condition)
  {
    currently = simulation_is::succeeded;
  } else {
    currently = simulation_is::ongoing;
  }

  auto make_context = [&]()
  {
    auto scenario_runner_context = [&]()
    {
      auto current = [&]()
      {
        boost::property_tree::ptree result {};

        result.put("SequenceName", (*sequence_manager_).current_sequence_name());
        result.put("EventName",    (*sequence_manager_).current_event_name());

        return result;
      };

      boost::property_tree::ptree result {};

      result.add_child("Current", current());
      result.add_child("Sequences", (*sequence_manager_).property());
      result.add_child("FailureConditions", failure.property());
      result.add_child("SuccessConditions", success.property());

      return result;
    };

    boost::property_tree::ptree result {};

    result.add_child("ScenarioRunnerContext", scenario_runner_context());

    return result;
  };

  std::stringstream ss {};
  boost::property_tree::write_json(ss, make_context());

  scenario_runner_msgs::msg::StringStamped message {};
  message.stamp = this->now();
  message.data = ss.str();

  // std::cout << message.data.c_str() << std::endl;

  publisher_->publish(message);
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to update simulation.");
}

}  // namespace scenario_runner
