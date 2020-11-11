#include <boost/cstdlib.hpp>
#include <exception>
#include <glog/logging.h>
#include <ros/ros.h>
#include <scenario_logger/logger.h>
#include <scenario_logger/simple_logger.hpp>
#include <scenario_runner/scenario_runner.h>
#include <scenario_runner/scenario_terminater.h>
#include <signal.h>
#include <thread>

static scenario_runner::ScenarioTerminator terminator { "0.0.0.0", 10000 };

auto to_signal_name = [](int signal) -> std::string
{
  switch (signal)
  {
    case SIGABRT: return "SIGABRT";
    case SIGFPE:  return "SIGPE";
    case SIGILL:  return "SIGILL";
    case SIGINT:  return "SIGINT";
    case SIGSEGV: return "SIGSEGV";
    case SIGTERM: return "SIGTERM";
    default:      return "";
  }
};

static void abort(int signal)
{
  ros::shutdown();

  const auto signal_name { to_signal_name(signal) };

  if (signal_name.empty())
  {
    SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Simulation failed unexpectedly");
  }
  else
  {
    SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Simulation failed unexpectedly (" << signal_name << ")");
  }
  scenario_logger::log.write();

  if (signal_name.empty())
  {
    LOG_SIMPLE(error() << "Abort simulation");
  }
  else
  {
    LOG_SIMPLE(error() << "Abort simulation (" << signal_name << ")");
  }

  std::quick_exit(EXIT_SUCCESS);
}

int main(int argc, char * argv[]) try
{
  scenario_logger::slog.open("/tmp/log", std::ios::trunc);

  ros::init(argc, argv, "scenario_runner_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter([](const char*, int)
  {
    return abort(0);
  });

  struct sigaction action {};
  memset(&action, 0, sizeof(struct sigaction));
  sigemptyset(&action.sa_mask);
  action.sa_flags |= SA_SIGINFO;
  action.sa_handler = &abort;

  if (sigaction(SIGINT, &action, nullptr) < 0)
  {
    LOG_SIMPLE(error() << "Failed to install SIGINT handler");
  }

  scenario_logger::log.setStartDatetime(ros::Time::now());
  SCENARIO_LOG_STREAM(CATEGORY("simulation", "progress"), "Logging started.");

  std::string scenario_id;
  pnh.getParam("scenario_id", scenario_id);
  scenario_logger::log.setScenarioID(scenario_id);
  LOG_SIMPLE(info() << "Scenario ID: " << scenario_id);

  std::string log_output_path;
  pnh.getParam("log_output_path", log_output_path);
  scenario_logger::log.setLogOutputPath(log_output_path);

  LOG_SIMPLE(info() << "Sleep for 10 seconds");
  for (auto i { 10 }; 0 < i; std::this_thread::sleep_for(std::chrono::seconds(1)))
  {
    LOG_SIMPLE(info() << "Sleeping... " << --i);
  }
  LOG_SIMPLE(info() << "Wake-up");

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

  // LOG_SIMPLE(error() << "ros::ok() == false");

  // if (runner.currently == simulation_is::ongoing)
  // {
  //   SCENARIO_INFO_STREAM(CATEGORY(), "Simulation aborted.");
  //   scenario_logger::log.write();
  //   terminator.sendTerminateRequest(boost::exit_failure);
  //   return boost::exit_failure;
  // }
  // else
  // {
  //   SCENARIO_INFO_STREAM(CATEGORY(), "Simulation unexpectedly failed.");
  //   scenario_logger::log.write();
  //   return boost::exit_exception_failure;
  // }

  return EXIT_FAILURE;
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
