#include <chrono>
#include <thread>

#include <scenario_runner_msgs/StringStamped.h>

#include <scenario_logger/logger.h>
#include <scenario_logger/simple_logger.hpp>
#include <scenario_runner/scenario_runner.h>

namespace scenario_runner
{
ScenarioRunner::ScenarioRunner(ros::NodeHandle nh, ros::NodeHandle pnh)
  : currently { simulation_is::ongoing }
  ,  nh_ {  nh }
  , pnh_ { pnh }
  , publisher_ { pnh.advertise<scenario_runner_msgs::StringStamped>("context", 1) }
  , simulator_ { std::make_shared<ScenarioAPI>() }
{
  using scenario_logger::slog;
  using scenario_logger::endlog;

  pnh_.getParam("scenario_path", scenario_path_);

  if (not (*simulator_).waitAutowareInitialize())
  {
    SCENARIO_ERROR_THROW(CATEGORY(), "Failed to initialize Autoware.");
  }

  try
  {
    slog.info() << "Loading scenario " << scenario_path_ << endlog;
    scenario_ = YAML::LoadFile(scenario_path_);
    slog.info() << "Success to load the scenario" << endlog;
  }
  catch (...)
  {
    slog.info() << "Failed to load the scenario" << endlog;
    SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to load YAML file \"" << scenario_path_ << "\".");
  }
}

void ScenarioRunner::run()
try
{
  using scenario_logger::slog;
  using scenario_logger::endlog;

  context.define(simulator_);

  slog.info() << "Parsing start" << endlog;

  slog.info() << "Parsing 'Entity'" << endlog;
  call_with_essential(scenario_, "Entity", [&](const auto& node)
  {
    context.define(
      entity_manager_ =
        std::make_shared<scenario_entities::EntityManager>(
          node, simulator_));
  });

  slog.info() << "Parsing 'Intersections'" << endlog;
  call_with_optional(scenario_, "Intersection", [&](const auto& node)
  {
    context.define(
      intersection_manager_ =
        std::make_shared<scenario_intersection::IntersectionManager>(
          node, simulator_));
  });

  slog.info() << "Parsing 'Story'" << endlog;
  call_with_essential(scenario_, "Story", [&](const auto& node)
  {
    slog.info() << "Setting 'Story' to entities" << endlog;
    context.entities().setStory(node);

    slog.info() << "Initializing entities" << endlog;
    context.entities().initialize();

    slog.info() << "Parsing 'Story.Init'" << endlog;
    call_with_essential(node, "Init", [&](const auto& node)
    {
      slog.info() << "Parsing 'Story.Init.Intersection'" << endlog;
      call_with_optional(node, "Intersection", [&](const auto& node)
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

    slog.info() << "Parsing 'Story.Act'" << endlog;
    sequence_manager_ =
      std::make_shared<scenario_sequence::SequenceManager>(
        context, node["Act"]);

    slog.info() << "Parsing 'Story.EndCondition'" << endlog;
    call_with_essential(node, "EndCondition", [&](const auto& node)
    {
      slog.info() << "Parsing 'Story.EndCondition.Success'" << endlog;
      call_with_optional(node, "Success", [&](const auto& node)
      {
        success = scenario_expression::read(context, node);
        // SCENARIO_INFO_STREAM(CATEGORY(), "Loaded success condition: " << success);
      });

      slog.info() << "Parsing 'Story.EndCondition.Failure'" << endlog;
      call_with_optional(node, "Failure", [&](const auto& node)
      {
        failure = scenario_expression::read(context, node);
        // SCENARIO_INFO_STREAM(CATEGORY(), "Loaded failure condition: " << failure);
      });
    });
  });

  slog.info() << "Parsing completed. There is no syntax-error" << endlog;

  slog.info() << "Waiting for simulation APIs to be ready" << endlog;
  simulator_->waitAPIReady();
  slog.info() << "Simulation APIs are ready" << endlog;

  timer_ = nh_.createTimer(ros::Duration(0.01), &ScenarioRunner::update, this);

  slog.info() << "Sending engage to Autoware" << endlog;
  if (not simulator_->sendEngage(true))
  {
    SCENARIO_ERROR_THROW(CATEGORY(), "Failed to send engage.");
  }
  slog.info() << "Engaged" << endlog;

  scenario_logger::log.initialize(ros::Time::now()); // NOTE: initialize logger's clock here.
  slog.info() << "Clock initialized" << endlog;

  slog.info() << "Scenario parsed" << endlog;
  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "Simulation started.");
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize ScenarioRunner.");
}

void ScenarioRunner::update(const ros::TimerEvent & event) try
{
  context.json << (indent++) << "{\n";
  context.json << (indent++) << std::quoted("ScenarioRunnerContext") << ": {\n";

  scenario_logger::log.updateMoveDistance(simulator_->getMoveDistance());

  (*sequence_manager_).update(intersection_manager_);

  // currently = (*entity_manager_).update(intersection_manager_);

  const auto fulfilled_failure_condition { failure.evaluate(context) };
  const auto fulfilled_success_condition { success.evaluate(context) };

  context.json << (indent++) << std::quoted("FailureConditions") << ": [\n";
  context.json << failure;
  context.json << (--indent) << "],\n";

  context.json << (indent++) << std::quoted("SuccessConditions") << ": [\n";
  context.json << success;
  context.json << (--indent) << "],\n";

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

  context.json << (indent++) << std::quoted("Current") << ": {\n";
  context.json << indent << std::quoted("SequenceName") << ": " << std::quoted((*sequence_manager_).current_sequence_name()) << ",\n";
  context.json << indent << std::quoted("EventName") << ": " << std::quoted((*sequence_manager_).current_event_name()) << "\n";
  context.json << (--indent) << "}\n";

  context.json << (--indent) << "}\n";
  context.json << (--indent) << "}," << std::endl;

  scenario_runner_msgs::StringStamped message {};
  message.header.stamp = ros::Time::now();
  message.data = context.json.str();

  std::cout << message.data.c_str() << std::endl;

  publisher_.publish(message);

  std::stringstream ss {};
  std::swap(context.json, ss);
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to update simulation.");
}

}  // namespace scenario_runner
