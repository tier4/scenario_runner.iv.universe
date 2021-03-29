// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "glog/logging.h"
#include "scenario_logger/logger.hpp"
#include "scenario_logger/simple_logger.hpp"
#include "scenario_runner/scenario_runner.hpp"
#include "boost/cstdlib.hpp"
#include "boost/filesystem.hpp"
#include "boost/filesystem/operations.hpp"
#include <exception>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"

static void failureCallback()
{
  SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Simulation failed unexpectedly.");
  scenario_logger::log.write();
}

void dump_diagnostics(const std::string & path, double mileage, double duration, int exit_code)
{
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

auto to_signal_name = [](int signal) -> std::string
{
  switch (signal)
  {
    case SIGABRT: return "SIGABRT";
    case SIGFPE:  return "SIGFPE";
    case SIGILL:  return "SIGILL";
    case SIGINT:  return "SIGINT";
    case SIGSEGV: return "SIGSEGV";
    case SIGTERM: return "SIGTERM";
    default:      return "";
  }
};

static void abort(int signal)
{
  rclcpp::shutdown();

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

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureFunction(&failureCallback);
  google::InstallFailureWriter([](const char*, int)
  {
    return abort(0);
  });

  struct sigaction action {};
  memset(&action, 0, sizeof(struct sigaction));
  sigemptyset(&action.sa_mask);
  action.sa_flags |= SA_SIGINFO;
  action.sa_handler = &abort;
  sigaction(SIGINT, &action, nullptr);

  /*
  * setup scenario runner
  */

  rclcpp::init(argc, argv);

  const auto runner_ptr = std::make_shared<scenario_runner::ScenarioRunner>();

  scenario_logger::log.setStartDatetime(runner_ptr->now());
  SCENARIO_LOG_STREAM(CATEGORY("simulation", "progress"), "Logging started.");

  std::string scenario_id{runner_ptr->declare_parameter("scenario_id").get<std::string>()};
  scenario_logger::log.setScenarioID(scenario_id);
  boost::filesystem::create_directory("/tmp/scenario_runner_node");
  scenario_logger::slog.open("/tmp/scenario_runner_node/" + scenario_id + ".json", std::ios::trunc);

  std::string log_output_path{runner_ptr->declare_parameter("log_output_path").get<std::string>()};
  scenario_logger::log.setLogOutputPath(log_output_path);

  LOG_SIMPLE(info() << "Sleep for 10 seconds");
  for (auto i { 10 }; 0 < i; std::this_thread::sleep_for(std::chrono::seconds(1)))
  {
    LOG_SIMPLE(info() << "Sleeping... " << --i);
  }
  LOG_SIMPLE(info() << "Wake-up");
  const auto & path = runner_ptr->declare_parameter("json_dump_path").get<std::string>();
  const auto dump = [&runner_ptr, path](int exit_code) {
      dump_diagnostics(
        path, runner_ptr->current_mileage(),
        (runner_ptr->now() - scenario_logger::log.begin()).seconds(), exit_code);
    };

  try {
    /*
  * start simulation
  */
    for (runner_ptr->run(); rclcpp::ok(); rclcpp::spin_some(runner_ptr)) {
      runner_ptr->spin_simulator();
      static auto previously{runner_ptr->currently};

      if (previously != runner_ptr->currently) {
        switch (runner_ptr->currently) {
          case simulation_is::succeeded:
            SCENARIO_INFO_STREAM(CATEGORY("simulator", "endcondition"), "simulation succeeded");
            scenario_logger::log.write();
            {
              const auto ret = boost::exit_success;
              dump(ret);
              rclcpp::shutdown();
              return ret;
            }

          case simulation_is::failed:
            SCENARIO_INFO_STREAM(CATEGORY("simulator", "endcondition"), "simulation failed");
            scenario_logger::log.write();
            {
              const auto ret = boost::exit_test_failure;
              dump(ret);
              rclcpp::shutdown();
              return ret;
            }

          case simulation_is::ongoing:
            break;
        }
        previously = runner_ptr->currently;
      }
    }

    if (runner_ptr->currently == simulation_is::ongoing) {
      SCENARIO_INFO_STREAM(CATEGORY(), "Simulation aborted.");
      scenario_logger::log.write();
      const auto ret = boost::exit_failure;
      dump(ret);
      rclcpp::shutdown();
      return ret;
    } else {
      SCENARIO_INFO_STREAM(CATEGORY(), "Simulation unexpectedly failed.");
      scenario_logger::log.write();
      rclcpp::shutdown();
      return boost::exit_exception_failure;
    }

    // return EXIT_FAILURE;
  } catch (const std::exception & e) {
    SCENARIO_ERROR_STREAM(
      CATEGORY(
        "simulator",
        "endcondition"),
      "Unexpected standard exception thrown: " << e.what());
    scenario_logger::log.write();
    dump(boost::exit_exception_failure);
  } catch (...) {
    SCENARIO_ERROR_STREAM(
      CATEGORY(
        "simulator",
        "endcondition"), "Unexpected non-standard exception thrown.");
    scenario_logger::log.write();
    dump(boost::exit_exception_failure);
  }
  rclcpp::shutdown();
  return 0;
}
