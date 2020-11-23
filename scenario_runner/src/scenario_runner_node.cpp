#include <glog/logging.h>
#include <scenario_logger/logger.h>
#include <scenario_runner/scenario_runner.h>
#include <boost/cstdlib.hpp>
#include <exception>
#include <rclcpp/rclcpp.hpp>

static void failureCallback()
{
  SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Simulation failed unexpectedly.");
  scenario_logger::log.write();
}

void dump_diagnostics(const std::string & path, double mileage, double duration, int exit_code){
  boost::property_tree::ptree pt;
  pt.put("code", exit_code);
  pt.put("duration", duration);
  pt.put("mileage", mileage);

  switch (exit_code) {
    case boost::exit_success:
      pt.put("message", "exit_success");
      break;
    case boost::exit_failure:
      pt.put("message", "exit_failure");
      break;
    case boost::exit_test_failure:
      pt.put("message", "exit_test_failure");
      break;
    case boost::exit_exception_failure:
      pt.put("message", "exit_exception_failure");
      break;
    default:
      pt.put("message", "unknown_exit_code");
  }
  boost::property_tree::write_json(path, pt);
}

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureFunction(&failureCallback);

  /*
  * setup scenario runner
  */

  rclcpp::init(argc, argv);

  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "ScenarioRunner instantiated.");
  const auto runner_ptr = std::make_shared<scenario_runner::ScenarioRunner>();

  scenario_logger::log.setStartDatetime(runner_ptr->now());
  SCENARIO_LOG_STREAM(CATEGORY("simulation", "progress"), "Logging started.");

  std::string scenario_id{runner_ptr->declare_parameter("scenario_id").get<std::string>()};
  scenario_logger::log.setScenarioID(scenario_id);

  std::string log_output_path{runner_ptr->declare_parameter("log_output_path").get<std::string>()};
  scenario_logger::log.setLogOutputPath(log_output_path);

  SCENARIO_INFO_STREAM(CATEGORY(), "Sleep for 10 seconds.");
  std::this_thread::sleep_for(std::chrono::seconds { 10 });
  SCENARIO_INFO_STREAM(CATEGORY(), "Wake-up.");
  const auto & path = runner_ptr->declare_parameter("json_dump_path").get<std::string>();
  const auto dump = [&runner_ptr, path](int exit_code){
    dump_diagnostics(path, runner_ptr->current_mileage(),
                     (runner_ptr->now() -  scenario_logger::log.begin()).seconds(), exit_code);
  };

  try{
    /*
  * start simulation
  */
    for (runner_ptr->run(); rclcpp::ok(); rclcpp::spin_some(runner_ptr))
    {
      static auto previously{runner_ptr->currently};

      if (previously != runner_ptr->currently)
      {
        switch (runner_ptr->currently)
        {
          case simulation_is::succeeded:
          SCENARIO_INFO_STREAM(CATEGORY("simulator", "endcondition"), "simulation succeeded");
             scenario_logger::log.write();
            {
              const auto ret = boost::exit_success;
              dump(ret);
              return ret;
            }

          case simulation_is::failed:
          SCENARIO_INFO_STREAM(CATEGORY("simulator", "endcondition"), "simulation failed");
             scenario_logger::log.write();
            {
              const auto ret = boost::exit_test_failure;
              dump(ret);
              return ret;
            }

          case simulation_is::ongoing:
            break;
        }
        previously = runner_ptr->currently;
      }
    }

    if (runner_ptr->currently == simulation_is::ongoing)
    {
      SCENARIO_INFO_STREAM(CATEGORY(), "Simulation aborted.");
      scenario_logger::log.write();
      const auto ret = boost::exit_failure;
      dump(ret);
      return ret;
    }
    else
    {
      SCENARIO_INFO_STREAM(CATEGORY(), "Simulation unexpectedly failed.");
      scenario_logger::log.write();
      return boost::exit_exception_failure;
    }
  } catch (const std::exception& e) {
    SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Unexpected standard exception thrown: " << e.what());
    scenario_logger::log.write();
    dump(boost::exit_exception_failure);
  } catch (...)
  {
    SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Unexpected non-standard exception thrown.");
    scenario_logger::log.write();
    dump(boost::exit_exception_failure);
  }
}