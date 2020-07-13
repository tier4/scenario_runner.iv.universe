#include <scenario_logger/logger.h>
#include <scenario_runner/scenario_runner.h>

namespace scenario_runner
{
ScenarioRunner::ScenarioRunner(ros::NodeHandle nh, ros::NodeHandle pnh)
: currently{simulation_is::ongoing},
  nh_{nh},
  pnh_{pnh},
  simulator_{std::make_shared<ScenarioAPI>()}
{
  pnh_.getParam("scenario_path", scenario_path_);

  (*simulator_).waitAutowareInitialize();

  try
  {
    scenario_ = YAML::LoadFile(scenario_path_);
  }
  catch (...)
  {
    SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to load YAML file \"" << scenario_path_ << "\".");
  }
}

void ScenarioRunner::run()
try
{
  context.define(simulator_);

  call_with_essential(scenario_, "Entity", [&](const auto& node) mutable
  {
    context.define(
      entity_manager_ =
        std::make_shared<scenario_entities::EntityManager>(
          node, simulator_));
  });

  call_with_optional(scenario_, "Intersection", [&](const auto& node) mutable
  {
    context.define(
      intersection_manager_ =
        std::make_shared<scenario_intersection::IntersectionManager>(
          node, simulator_));
  });

  call_with_essential(scenario_, "Story", [&](const auto& node) mutable
  {
    context.entities->setStory(node);
    context.entities->initialize();

    call_with_essential(node, "Init", [&](const auto& node) mutable
    {
      call_with_optional(node, "Intersection", [&](const auto& node) mutable
      {
        context.intersections->initialize(node);
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

    call_with_essential(node, "EndCondition", [&](const auto& node) mutable
    {
      call_with_optional(node, "Success", [&](const auto& node) mutable
      {
        success = scenario_expression::read(context, node);
        SCENARIO_INFO_STREAM(CATEGORY(), "Loaded success condition: " << success);
      });

      call_with_optional(node, "Failure", [&](const auto& node) mutable
      {
        failure = scenario_expression::read(context, node);
        SCENARIO_INFO_STREAM(CATEGORY(), "Loaded failure condition: " << failure);
      });
    });
  });

  SCENARIO_INFO_STREAM(CATEGORY(), "Waiting for the simulator API to be ready.");
  simulator_->waitAPIReady();
  SCENARIO_INFO_STREAM(CATEGORY(), "Simulator API is ready.");

  timer_ = nh_.createTimer(ros::Duration(0.01), &ScenarioRunner::update, this);

  simulator_->sendEngage(true);
  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "ScenarioRunner engaged Autoware.");

  scenario_logger::log.initialize(ros::Time::now()); // NOTE: initialize logger's clock here.
  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "Simulation started.");
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize ScenarioRunner.");
}

void ScenarioRunner::update(const ros::TimerEvent & event)
{
  scenario_logger::log.updateMoveDistance(simulator_->getMoveDistance());
  (*sequence_manager_).update(intersection_manager_);

  // currently = (*entity_manager_).update(intersection_manager_);

  if (failure.evaluate(context))
  {
    currently = simulation_is::failed;
  }
  else if (success.evaluate(context))
  {
    currently = simulation_is::succeeded;
  }
  else
  {
    currently = simulation_is::ongoing;
  }
}

}  // namespace scenario_runner
