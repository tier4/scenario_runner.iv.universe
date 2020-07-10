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

  ROS_INFO_STREAM("\e[1;32mIntersection:\e[0m");
  context.define(
    intersection_manager_ =
      std::make_shared<scenario_intersection::IntersectionManager>(
        scenario_["Intersection"], simulator_));

  ROS_INFO_STREAM("\e[1;32mStory:\e[0m");
  entity_manager_->setStory(scenario_["Story"]);

  ROS_INFO_STREAM("\e[1;32m  Init:\e[0m");
  entity_manager_->initialize();
  intersection_manager_->initialize(scenario_["Story"]["Init"]["Intersection"]);

  ROS_INFO_STREAM("\e[1;32m  Act:\e[0m");
  sequence_manager_ =
    std::make_shared<scenario_sequence::SequenceManager>(
      context, scenario_["Story"]["Act"]);

  success = scenario_expression::read(context, scenario_["Story"]["EndCondition"]["Success"]);
  failure = scenario_expression::read(context, scenario_["Story"]["EndCondition"]["Failure"]);

  SCENARIO_LOG_STREAM(CATEGORY(), "Waiting for the simulator API to be ready.");
  simulator_->waitAPIReady();
  SCENARIO_LOG_STREAM(CATEGORY(), "Simulator API is ready.");

  timer_ = nh_.createTimer(ros::Duration(0.01), &ScenarioRunner::update, this);

  simulator_->sendEngage(true);
  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "ScenarioRunner engaged Autoware.");

  scenario_logger::log.begin(); // NOTE: initialize logger's clock here.
  SCENARIO_LOG_STREAM(CATEGORY("simulation", "progress"), "Simulation started.");
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

  ROS_INFO_STREAM("\e[1;36m  SimulationIs: " << currently << "\n");
}

}  // namespace scenario_runner
