#include <boost/cstdlib.hpp>
#include <exception>
#include <glog/logging.h>
#include <ros/ros.h>
#include <scenario_logger/logger.h>
#include <scenario_runner/scenario_runner.h>
#include <scenario_runner/scenario_terminater.h>


namespace scenario_runner
{
static void failureCallback() { scenario_logger::log.write(); }

static void failureCallback() { scenario_logger::log.write(); }

int main(int argc, char * argv[]) try
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureFunction(&failureCallback);

  ros::init(argc, argv, "scenario_runner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  scenario_logger::log.setStartDatetime(ros::Time::now());
  SCENARIO_LOG_STREAM(CATEGORY("simulation", "progress"), "Logging started.");

  std::string scenario_id;
  pnh.getParam("scenario_id", scenario_id);
  scenario_logger::log.setScenarioID(scenario_id);

  std::string log_output_path;
  pnh.getParam("log_output_path", log_output_path);
  scenario_logger::log.setLogOutputPath(log_output_path);

  /**
   * setup scenario runner
   */
  scenario_runner::ScenarioRunner runner(nh, pnh);

  /**
   * setup scenario logger
   */
  scenario_logger::log.setStartDatetime(ros::Time::now());

  std::string scenario_id;
  pnh.getParam("scenario_id", scenario_id);
  scenario_logger::log.setScenarioID(scenario_id);

  std::string log_output_path;
  pnh.getParam("log_output_path", log_output_path);
  scenario_logger::log.setLogOutputPath(log_output_path);

  /**
   * start simulation
   */
  for (runner.run(); ros::ok(); ros::spinOnce()) {
      static auto previously{runner.currently};

      if (previously != runner.currently) {
        switch (previously = runner.currently) {
          case simulation_is::succeeded:
            scenario_logger::log.addLog(
              scenario_logger_msgs::Level::LEVEL_INFO, {"simulator", "endcondition"},
              "simulation succeeded", "scenario_runner");
            runner.terminate();
            break;

          case simulation_is::failed:
            scenario_logger::log.addLog(
              scenario_logger_msgs::Level::LEVEL_INFO, {"simulator", "endcondition"},
              "simulation failed", "scenario_runner");
            runner.terminate();
            break;

          default:
          case simulation_is::ongoing:
            scenario_logger::log.addLog(
              scenario_logger_msgs::Level::LEVEL_LOG, {"simulator", "endcondition"},
              "simulation ongoing", "scenario_runner");
            break;
        }
      }
    }

  scenario_logger::log.write();

  return boost::exit_success;
}

catch (const std::exception& e)
{
  SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Unexpected standard exception thrown: " << e.what());
  scenario_logger::log.write();
  terminator.sendTerminateRequest(boost::exit_exception_failure);
}

catch (...)
{
  SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Unexpected non-standard exception thrown.");
  scenario_logger::log.write();
  terminator.sendTerminateRequest(boost::exit_exception_failure);
}
