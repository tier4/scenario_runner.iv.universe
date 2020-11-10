#include <boost/cstdlib.hpp>
#include <exception>
#include <glog/logging.h>
#include <ros/ros.h>
#include <scenario_logger/logger.h>
#include <scenario_logger/simple_logger.hpp>
#include <scenario_runner/scenario_runner.h>
#include <scenario_runner/scenario_terminater.h>
#include <thread>


static scenario_runner::ScenarioTerminator terminator { "0.0.0.0", 10000 };

static void failureCallback()
{
  SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Simulation failed unexpectedly.");
  scenario_logger::log.write();
}

int main(int argc, char * argv[]) try
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureFunction(&failureCallback);

  using scenario_logger::slog;
  using scenario_logger::endlog;

  slog.open("/tmp/log", std::ios::trunc);

  slog.info() << "test" << endlog;

  ros::init(argc, argv, "scenario_runner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  scenario_logger::log.setStartDatetime(ros::Time::now());
  SCENARIO_LOG_STREAM(CATEGORY("simulation", "progress"), "Logging started.");

  std::string scenario_id;
  pnh.getParam("scenario_id", scenario_id);
  scenario_logger::log.setScenarioID(scenario_id);
  slog.info() << "Scenario ID is " << scenario_id << endlog;

  std::string log_output_path;
  pnh.getParam("log_output_path", log_output_path);
  scenario_logger::log.setLogOutputPath(log_output_path);

  slog.info() << "Sleep for 10 seconds" << endlog;
  for (auto i { 10 }; 0 < i; --i)
  {
    slog.info() << "Sleeping... " << i << endlog;
    std::this_thread::sleep_for(std::chrono::seconds { 1 });
  }
  slog.info() << "Wake-up." << endlog;

  /*
   * setup scenario runner
   */
  scenario_runner::ScenarioRunner runner(nh, pnh);

  /*
   * start simulation
   */
  for (runner.run(); ros::ok(); ros::spinOnce())
  {
    static auto previously{runner.currently};

    if (previously != runner.currently)
    {
      switch (previously = runner.currently)
      {
      case simulation_is::succeeded:
        SCENARIO_INFO_STREAM(CATEGORY("simulator", "endcondition"), "simulation succeeded");
        scenario_logger::log.write();
        terminator.sendTerminateRequest(boost::exit_success);
        return boost::exit_success;

      case simulation_is::failed:
        SCENARIO_INFO_STREAM(CATEGORY("simulator", "endcondition"), "simulation failed");
        scenario_logger::log.write();
        terminator.sendTerminateRequest(boost::exit_test_failure);
        return boost::exit_test_failure;

      case simulation_is::ongoing:
        break;
      }
    }

    terminator.update_mileage(runner.current_mileage());

    terminator.update_duration(
      (ros::Time::now() - scenario_logger::log.begin()).toSec());
  }

  if (runner.currently == simulation_is::ongoing)
  {
    SCENARIO_INFO_STREAM(CATEGORY(), "Simulation aborted.");
    scenario_logger::log.write();
    terminator.sendTerminateRequest(boost::exit_failure);
    return boost::exit_failure;
  }
  else
  {
    SCENARIO_INFO_STREAM(CATEGORY(), "Simulation unexpectedly failed.");
    scenario_logger::log.write();
    return boost::exit_exception_failure;
  }
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
