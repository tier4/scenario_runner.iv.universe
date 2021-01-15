#include <chrono>
#include <regex>
#include <thread>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <scenario_logger/logger.h>
#include <scenario_logger/simple_logger.hpp>
#include <scenario_runner/scenario_runner.h>
#include <scenario_runner_msgs/StringStamped.h>

namespace scenario_runner
{
ScenarioRunner::ScenarioRunner(ros::NodeHandle nh, ros::NodeHandle pnh)
  : currently { simulation_is::ongoing }
  ,  nh_ {  nh }
  , pnh_ { pnh }
  , publisher_ { pnh.advertise<scenario_runner_msgs::StringStamped>("context", 1) }
  , simulator_ { std::make_shared<ScenarioAPI>() }
{
  pnh_.getParam("scenario_path", scenario_path_);

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

void ScenarioRunner::run() try
{
  LOG_SIMPLE(info() << "Parse scenario");
  context.define(simulator_);

  LOG_SIMPLE(info() << "Parse 'Entity'");
  call_with_essential(scenario_, "Entity", [&](const auto& node)
  {
    context.define(
      entity_manager_ =
        std::make_shared<scenario_entities::EntityManager>(
          node, simulator_));
  });

  LOG_SIMPLE(info() << "Parse 'Intersections'");
  call_with_optional(scenario_, "Intersection", [&](const auto& node)
  {
    context.define(
      intersection_manager_ =
        std::make_shared<scenario_intersection::IntersectionManager>(
          node, simulator_));
  });

  LOG_SIMPLE(info() << "Parse 'Story'");
  call_with_essential(scenario_, "Story", [&](const auto& node)
  {
    LOG_SIMPLE(info() << "Set 'Story' to entities");
    context.entities().setStory(node);

    LOG_SIMPLE(info() << "Initializing entities");
    context.entities().initialize();

    LOG_SIMPLE(info() << "Parse 'Story.Init'");
    call_with_essential(node, "Init", [&](const auto& node)
    {
      LOG_SIMPLE(info() << "Parse 'Story.Init.Intersection'");
      call_with_optional(node, "Intersection", [&](const auto& node)
      {
        context.intersections().initialize(node);
      });
    });

    LOG_SIMPLE(info() << "Parse 'Story.Act'");
    sequence_manager_ =
      std::make_shared<scenario_sequence::SequenceManager>(
        context, node["Act"]);

    LOG_SIMPLE(info() << "Parse 'Story.EndCondition'");
    call_with_essential(node, "EndCondition", [&](const auto& node)
    {
      LOG_SIMPLE(info() << "Parse 'Story.EndCondition.Success'");
      call_with_optional(node, "Success", [&](const auto& node)
      {
        success = scenario_expression::read(context, node);
      });

      LOG_SIMPLE(info() << "Parse 'Story.EndCondition.Failure'");
      call_with_optional(node, "Failure", [&](const auto& node)
      {
        failure = scenario_expression::read(context, node);
      });
    });
  });

  LOG_SIMPLE(info() << "Parse completed. There is no syntax-error");

  LOG_SIMPLE(info() << "Waiting for simulation APIs to be ready");
  simulator_->waitAPIReady();
  LOG_SIMPLE(info() << "Simulation APIs are ready");

  timer_ = nh_.createTimer(ros::Duration(0.05), &ScenarioRunner::update, this);

  LOG_SIMPLE(info() << "Engage!");
  simulator_->sendEngage(true);

  LOG_SIMPLE(info() << "Initialize SimulationTime");
  scenario_logger::log.initialize(ros::Time::now()); // NOTE: initialize logger's clock here.

  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "Simulation started.");
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize ScenarioRunner.");
}

void ScenarioRunner::update(const ros::TimerEvent & event) try
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
  }
  else
  {
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

  static const std::regex pattern { R"(\[\s+\"\"\s+\])" };

  scenario_runner_msgs::StringStamped message {};
  message.header.stamp = ros::Time::now();
  message.data = std::regex_replace(ss.str(), pattern, R"([])"); // XXX HACK

  // std::cout << message.data.c_str() << std::endl;

  publisher_.publish(message);
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to update simulation.");
}

}  // namespace scenario_runner
