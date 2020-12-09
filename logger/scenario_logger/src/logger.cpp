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

#include "scenario_logger/logger.hpp"

#include <string>
#include <vector>
#include <utility>

namespace scenario_logger
{

static int schwarz_counter {0};

static typename std::aligned_storage<sizeof(Logger), alignof(Logger)>::type memory;

Logger & log {reinterpret_cast<Logger &>(memory)};

LoggerInitializer::LoggerInitializer()
{
  if (schwarz_counter++ == 0) {
    new (&log) Logger();
  }
}

LoggerInitializer::~LoggerInitializer()
{
  if (--schwarz_counter == 0) {
    log.~Logger();
  }
}

Logger::Logger()
: data_{},
  log_output_path_{boost::none},
  rclcpp_logger_(rclcpp::get_logger("scenario_logger"))
{}

void Logger::setStartDatetime(const rclcpp::Time & time)
{
  data_.metadata.start_datetime = toIso6801(time);
}

void Logger::setScenarioID(const std::string & id)
{
  data_.metadata.scenario_id = id;
}

void Logger::setLogOutputPath(const std::string & directory)
{
  log_output_path_ = directory;
}

const rclcpp::Time & Logger::initialize(const rclcpp::Time & time)
{
  return time_ = time;
}

const rclcpp::Time & Logger::begin() const
{
  return time_;
}

std::string toIso6801(const rclcpp::Time & stamp)
{
  const boost::posix_time::time_duration duration(0, 0, 0, stamp.nanoseconds());
  const boost::posix_time::ptime boost_time(boost::gregorian::date(1970, 1, 1), duration);
  return boost::posix_time::to_iso_extended_string(boost_time);
}

void Logger::write()
{
  if (log_output_path_) {
    const rclcpp::Time now {rclcpp::Clock(time_.get_clock_type()).now()};

    data_.metadata.end_datetime = toIso6801(now);
    data_.metadata.duration = (now - begin()).seconds();

    boost::property_tree::write_json(log_output_path_.get(), toJson(data_, rclcpp_logger_));
  } else {
    SCENARIO_ERROR_THROW(CATEGORY(), "No log_output_path specified.");
  }
}

void Logger::append(const scenario_logger_msgs::msg::Log & log)
{
  data_.log.push_back(log);
}

void Logger::append(
  int level,
  const std::vector<std::string> & categories,
  const std::string & description,
  const std::string & from)
{
  scenario_logger_msgs::msg::Log log;

  log.elapsed_time = rclcpp::Clock(time_.get_clock_type()).now() - begin();
  log.level.level = level;
  log.categories = categories;
  log.description = description;
  log.source = from;

  append(log);
}

std::size_t Logger::getNumberOfLog() const
{
  return data_.log.size();
}

void Logger::updateMoveDistance(float move_distance)
{
  data_.metadata.move_distance = move_distance;
}

boost::optional<boost::property_tree::ptree> toJson(const scenario_logger_msgs::msg::Log & data)
{
  using boost::property_tree::ptree;
  ptree pt;
  pt.put("elapsed_time", rclcpp::Duration(data.elapsed_time).seconds());
  switch (data.level.level) {
    case scenario_logger_msgs::msg::Level::LEVEL_LOG:
      {
        pt.put("level", "log");
        break;
      }
    case scenario_logger_msgs::msg::Level::LEVEL_INFO:
      {
        pt.put("level", "info");
        break;
      }
    case scenario_logger_msgs::msg::Level::LEVEL_WARN:
      {
        pt.put("level", "warn");
        break;
      }
    case scenario_logger_msgs::msg::Level::LEVEL_ERROR:
      {
        pt.put("level", "error");
        break;
      }
    default:
      {
        return boost::none;
      }
  }
  ptree children;

  for (const auto & each : data.categories) {
    ptree child {};
    child.put("", each);
    children.push_back(std::make_pair("", child));
  }

  pt.add_child("categories", children);
  pt.put("description", data.description);
  pt.put("from", data.source);

  return pt;
}

boost::property_tree::ptree toJson(const scenario_logger_msgs::msg::MetaData & data)
{
  boost::property_tree::ptree pt {};
  pt.put("scenario_id", data.scenario_id);
  pt.put("start_datetime", data.start_datetime);
  pt.put("end_datetime", data.end_datetime);
  pt.put("duration", data.duration);
  pt.put("autoware_commit_hash", data.autoware_commit_hash);
  pt.put("simulator_commit_hash", data.simulator_commit_hash);
  pt.put("move_distance", data.move_distance);
  return pt;
}

boost::property_tree::ptree toJson(
  const scenario_logger_msgs::msg::LoggedData & data, const
  rclcpp::Logger & rclcpp_logger)
{
  using boost::property_tree::ptree;
  ptree pt, log_tree;
  ptree metadata_tree = toJson(data.metadata);
  pt.add_child("metadata", metadata_tree);
  for (auto itr = data.log.begin(); itr != data.log.end(); itr++) {
    auto single_log_tree = toJson(*itr);
    if (single_log_tree) {
      // ptree child;
      // child.add_child("data",*single_log_tree);
      log_tree.push_back(std::make_pair("", *single_log_tree));
    } else {
      RCLCPP_ERROR_STREAM(rclcpp_logger, "failed to convert to json");
    }
  }
  pt.add_child("log", log_tree);
  return pt;
}

}  // namespace scenario_logger
