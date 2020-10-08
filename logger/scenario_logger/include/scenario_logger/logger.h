#ifndef SCENARIO_LOGGER_LOGGER_H_INCLUDED
#define SCENARIO_LOGGER_LOGGER_H_INCLUDED

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstdio>
#include <new>
#include <ros/ros.h>
#include <scenario_logger_msgs/LoggedData.h>
#include <sstream>

#define SCENARIO_LOG_FROM \
  std::string(__FILE__) + ":" + std::to_string(__LINE__)

#define SCENARIO_LOG_APPEND(LEVEL, CATEGORY, ...)                              \
  do                                                                           \
  {                                                                            \
    std::stringstream ss {};                                                   \
    ss << __VA_ARGS__;                                                         \
    scenario_logger::log.append(LEVEL, CATEGORY, ss.str(), SCENARIO_LOG_FROM); \
  }                                                                            \
  while (false)

#define CATEGORY(...) \
  std::vector<std::string>(std::initializer_list<std::string> {__VA_ARGS__})

#define SCENARIO_LOG_STREAM(CATEGORY, ...) \
  SCENARIO_LOG_APPEND( \
    scenario_logger_msgs::Level::LEVEL_LOG, CATEGORY, __VA_ARGS__)

#define SCENARIO_INFO_STREAM(CATEGORY, ...) \
  SCENARIO_LOG_APPEND( \
    scenario_logger_msgs::Level::LEVEL_INFO, CATEGORY, __VA_ARGS__); \
  ROS_INFO_STREAM(__VA_ARGS__)

#define SCENARIO_WARN_STREAM(CATEGORY, ...) \
  SCENARIO_LOG_APPEND( \
    scenario_logger_msgs::Level::LEVEL_WARN, CATEGORY, __VA_ARGS__); \
  ROS_WARN_STREAM(__VA_ARGS__)

#define SCENARIO_ERROR_STREAM(CATEGORY, ...) \
  SCENARIO_LOG_APPEND( \
    scenario_logger_msgs::Level::LEVEL_ERROR, CATEGORY, __VA_ARGS__); \
  ROS_ERROR_STREAM(__VA_ARGS__)

#define SCENARIO_ERROR_THROW(CATEGORY, ...)                                    \
  do                                                                           \
  {                                                                            \
    std::stringstream ss {};                                                   \
                                                                               \
    ss << __VA_ARGS__;                                                         \
                                                                               \
    scenario_logger::log.append(                                               \
      scenario_logger_msgs::Level::LEVEL_ERROR,                                \
      CATEGORY,                                                                \
      ss.str(),                                                                \
      SCENARIO_LOG_FROM);                                                      \
                                                                               \
    ROS_ERROR_STREAM(ss.str());                                                \
                                                                               \
    throw std::runtime_error { ss.str() };                                     \
  }                                                                            \
  while (false)

#define SCENARIO_ERROR_RETHROW(CATEGORY, ...)                                  \
  do                                                                           \
  {                                                                            \
    std::stringstream ss {};                                                   \
                                                                               \
    ss << __VA_ARGS__;                                                         \
                                                                               \
    scenario_logger::log.append(                                               \
      scenario_logger_msgs::Level::LEVEL_ERROR,                                \
      CATEGORY,                                                                \
      ss.str(),                                                                \
      SCENARIO_LOG_FROM);                                                      \
                                                                               \
    ROS_ERROR_STREAM(ss.str());                                                \
                                                                               \
    throw;                                                                     \
  }                                                                            \
  while (false)

#define SCENARIO_THROW_ERROR_ABOUT_INCOMPLETE_CONFIGURATION() \
  SCENARIO_ERROR_THROW(CATEGORY(), \
    "Failed to configure condition named '" << name_ << "' of type " << type_ << ".")

#define SCENARIO_LOG_ABOUT_TOGGLE_CONDITION_RESULT() \
  SCENARIO_LOG_STREAM(CATEGORY(), \
    "Changed value from " << std::boolalpha << result_ \
                << " to " << std::boolalpha << (not result_) \
                << " (Condition named '" << name_ << "' of type " << type_ << ").")

#define SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION() \
  SCENARIO_ERROR_RETHROW(CATEGORY(), \
    "Failed to configure Condition named '" << name_ << "' of type " << type_ << ".")

#define SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION() \
  SCENARIO_ERROR_RETHROW(CATEGORY(), \
    "Failed to configure Action named '" << name_ << "' of type " << type_ << ".")

#define SCENARIO_WARNING_ABOUT_NO_ACTORS_SPECIFIED() \
  SCENARIO_WARN_STREAM(CATEGORY(), \
    "No actors specified for Action named '" << name_ << "' of type " << type_ << ". " \
    "The action will cause no effects.\n\n" << node_ << "\n")

namespace scenario_logger
{

const ros::Time& begin();

std::string toIso6801(const ros::Time& stamp);

boost::property_tree::ptree toJson(const scenario_logger_msgs::MetaData& data);
boost::property_tree::ptree toJson(const scenario_logger_msgs::LoggedData& data);
boost::optional<boost::property_tree::ptree> toJson(const scenario_logger_msgs::Log& data);

class Logger
{
  scenario_logger_msgs::LoggedData data_;

  boost::optional<std::string> log_output_path_;

  ros::Time time_;

public:
  Logger();

  void setStartDatetime(const ros::Time&);
  void setScenarioID(const std::string&);
  void setLogOutputPath(const std::string& directory);

  const ros::Time& initialize(const ros::Time&);
  const ros::Time& begin() const;

  void write();

  void append(const scenario_logger_msgs::Log&);
  void append(int level,
              const std::vector<std::string>& categories,
              const std::string& description,
              const std::string& from);

  std::size_t getNumberOfLog() const;

  void updateMoveDistance(float move_distance);
};

extern Logger& log;

static struct LoggerInitializer
{
  LoggerInitializer();
  ~LoggerInitializer();
} initializer;

} // static Logger Log;

#endif  // SCENARIO_LOGGER_LOGGER_H_INCLUDED
