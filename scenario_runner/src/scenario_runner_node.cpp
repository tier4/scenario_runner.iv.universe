#include <boost/cstdlib.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <exception>
#include <glog/logging.h>
#include <ros/ros.h>
#include <scenario_logger/logger.h>
#include <scenario_logger/simple_logger.hpp>
#include <scenario_runner/scenario_runner.h>
#include <scenario_runner/scenario_terminater.h>
#include <signal.h>
#include <thread>

static scenario_runner::ScenarioTerminator terminator { "127.0.0.1", 10000 };

static void terminate(int signal)
{
  ros::shutdown();

  auto convert = [](int signal) -> std::string
  {
    switch (signal)
    {
      case SIGABRT: return " (SIGABRT)";
      case SIGFPE:  return " (SIGPE)";
      case SIGILL:  return " (SIGILL)";
      case SIGINT:  return " (SIGINT)";
      case SIGSEGV: return " (SIGSEGV)";
      case SIGTERM: return " (SIGTERM)";
      default:      return "";
    }
  };

  switch (terminator.status)
  {
  case boost::exit_success:
    SCENARIO_INFO_STREAM(CATEGORY("simulator", "endcondition"),
      "The simulation was succeeded (EndCondition.Success).");
    LOG_SIMPLE(info() << "The simulation was succeeded (EndCondition.Success).");
    break;

  case boost::exit_failure:
    SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"),
      "The simulation was terminated (given time-limit has been reached).");
    LOG_SIMPLE(error() << "The simulation was terminated (given time-limit has been reached).");
    break;

  case boost::exit_test_failure:
    SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"),
      "The simulation as failed (EndCondition.Failure).");
    LOG_SIMPLE(error() << "The simulation was failed (EndCondition.Failure).");
    break;

  case boost::exit_exception_failure:
    SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"),
      "The simulation was failed (syntax-error or internal-error of scenario_runner).");
    LOG_SIMPLE(error() << "The simulation was failed (syntax-error or internal-error of scenario_runner).");
    break;

  default:
    SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"),
      "The simulation was terminated unexpectedly " << convert(signal));
    LOG_SIMPLE(error() << "The simulation was terminated unexpectedly " << convert(signal));
    break;
  }

  scenario_logger::log.write();

  std::quick_exit(terminator.status);
}

int main(int argc, char * argv[]) try
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter([](const char*, int)
  {
    return terminate(0);
  });

  struct sigaction action {};
  memset(&action, 0, sizeof(struct sigaction));
  sigemptyset(&action.sa_mask);
  action.sa_flags |= SA_SIGINFO;
  action.sa_handler = &terminate;
  sigaction(SIGINT, &action, nullptr);

  ros::init(argc, argv, "scenario_runner_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  scenario_logger::log.setStartDatetime(ros::Time::now());
  SCENARIO_LOG_STREAM(CATEGORY("simulation", "progress"), "Logging started.");

  std::string scenario_id;
  pnh.getParam("scenario_id", scenario_id);
  scenario_logger::log.setScenarioID(scenario_id);
  boost::filesystem::create_directory("/tmp/scenario_runner_node");
  scenario_logger::slog.open("/tmp/scenario_runner_node/" + scenario_id + ".json", std::ios::trunc);

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
