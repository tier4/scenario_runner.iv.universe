#ifndef SCENARIO_LOGGER_LOGGER_H_INCLUDED
#define SCENARIO_LOGGER_LOGGER_H_INCLUDED

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstdio>
#include <cxxabi.h>
#include <new>
#include <ros/ros.h>
#include <scenario_logger_msgs/LoggedData.h>
#include <sstream>

#define ADD_LOG(level, categories, description) do {                          \
  std::stringstream ss;                                                       \
  int failed;                                                                 \
  ss << __FILE__　<< ":" << __LINE__ << ": ";                                 \
  ss << abi::__cxa_demangle(typeid(*this).name(), nullptr, nullptr, &failed)  \
     << "::" << __func__;                                                     \
  ss << " LINE : " << __LINE__;                                               \
  scenario_logger::log.addLog(level, categories, description, ss.str());  \
  } while(0)

#define STR(var) #var

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

public:
  Logger();

  void setStartDatetime(const ros::Time&);
  void setScenarioID(const std::string&);
  void setLogOutputPath(const std::string& directory);

  const ros::Time& begin() const;

  void write();

  void addLog(const scenario_logger_msgs::Log&);
  void addLog(int level,
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
