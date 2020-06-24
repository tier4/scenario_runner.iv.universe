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
  } catch (const std::exception & e) {
    ROS_ERROR_STREAM(e.what());

    scenario_logger::log.addLog(
      scenario_logger_msgs::Level::LEVEL_ERROR, {"simulator"},
      "failed to parse scenario : " + scenario_path_, "scenario_runner");
  }
}

void ScenarioRunner::run()
try
{
  context.define(simulator_);

  context.define(
    entity_manager_ =
      std::make_shared<scenario_entities::EntityManager>(
        scenario_["Entity"], simulator_));

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

  scenario_logger::log.addLog(
    scenario_logger_msgs::Level::LEVEL_LOG, {"simulator"}, "scenario runner start runnig",
    "scenario_runner");

  timer_ = nh_.createTimer(ros::Duration(0.01), &ScenarioRunner::update, this);

  if (false) // TEST CODE
  {
    auto e =
      scenario_expression::read(
        context,
        scenario_["Story"]["EndCondition"]["Experimental"]);

    ROS_ERROR_STREAM(e);
    ROS_ERROR_STREAM(e.evaluate(context));

    terminate();
  }
}

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
