#include <scenario_logger/logger.h>

namespace scenario_logger
{

static int schwarz_counter { 0 };

static typename std::aligned_storage<sizeof(Logger), alignof(Logger)>::type memory;

Logger& log { reinterpret_cast<Logger&>(memory) };

LoggerInitializer::LoggerInitializer()
{
  if (schwarz_counter++ == 0)
  {
    new (&log) Logger();
  }
}

LoggerInitializer::~LoggerInitializer()
{
  if (--schwarz_counter == 0)
  {
    log.~Logger();
  }
}

Logger::Logger()
  : data_ {}
  , log_output_path_ { boost::none }
{}

void Logger::setStartDatetime(const ros::Time& time)
{
  data_.metadata.start_datetime = toIso6801(time);
}

void Logger::setScenarioID(const std::string& id)
{
  data_.metadata.scenario_id = id;
}

void Logger::setLogOutputPath(const std::string& directory)
{
  log_output_path_ = directory;
}

const ros::Time& Logger::initialize(const ros::Time& time)
{
  return time_ = time;
}

const ros::Time& Logger::begin() const
{
  return time_;
}

std::string toIso6801(const ros::Time& stamp)
{
  return boost::posix_time::to_iso_extended_string(stamp.toBoost());
}

void Logger::write()
{
  if (log_output_path_)
  {
    const ros::Time now { ros::Time::now() };

    data_.metadata.end_datetime = toIso6801(now);
    data_.metadata.duration = (now - begin()).toSec();

    boost::property_tree::write_json(log_output_path_.get(), toJson(data_));
  }
  else
  {
    SCENARIO_ERROR_THROW(CATEGORY(), "No log_output_path specified.");
  }
}

void Logger::append(const scenario_logger_msgs::Log& log)
{
  data_.log.push_back(log);
}

void Logger::append(int level,
                    const std::vector<std::string>& categories,
                    const std::string& description,
                    const std::string& from)
{
  scenario_logger_msgs::Log log;

  log.elapsed_time = ros::Time::now() - begin();
  log.level.level = level;
  log.categories = categories;
  log.description = description;
  log.from = from;

  append(log);
}

std::size_t Logger::getNumberOfLog() const
{
  return data_.log.size();
};

void Logger::updateMoveDistance(float move_distance)
{
  data_.metadata.move_distance = move_distance;
}

boost::optional<boost::property_tree::ptree> toJson(const scenario_logger_msgs::Log& data)
{
  using namespace boost::property_tree;
  ptree pt;
  pt.put("elapsed_time", data.elapsed_time);
  switch (data.level.level)
  {
  case scenario_logger_msgs::Level::LEVEL_LOG:
    {
      pt.put("level","log");
      break;
    }
  case scenario_logger_msgs::Level::LEVEL_INFO:
    {
      pt.put("level","info");
      break;
    }
  case scenario_logger_msgs::Level::LEVEL_WARN:
    {
      pt.put("level","warn");
      break;
    }
  case scenario_logger_msgs::Level::LEVEL_ERROR:
    {
      pt.put("level","error");
      break;
    }
  default:
    {
      return boost::none;
    }
  }
  ptree children;

  for (const auto& each : data.categories)
  {
    ptree child {};
    child.put("", each);
    children.push_back(std::make_pair("", child));
  }

  pt.add_child("categories",children);
  pt.put("description",data.description);
  pt.put("from",data.from);

  return pt;
}

boost::property_tree::ptree toJson(const scenario_logger_msgs::MetaData& data)
{
  boost::property_tree::ptree pt {};
  pt.put("scenario_id",data.scenario_id);
  pt.put("start_datetime",data.start_datetime);
  pt.put("end_datetime",data.end_datetime);
  pt.put("duration",data.duration);
  pt.put("autoware_commit_hash",data.autoware_commit_hash);
  pt.put("simulator_commit_hash",data.simulator_commit_hash);
  pt.put("move_distance",data.move_distance);
  return pt;
}

boost::property_tree::ptree toJson(const scenario_logger_msgs::LoggedData& data)
{
  using namespace boost::property_tree;
  ptree pt,log_tree;
  ptree metadata_tree = toJson(data.metadata);
  pt.add_child("metadata",metadata_tree);
  for(auto itr = data.log.begin(); itr != data.log.end(); itr++)
  {
    auto single_log_tree = toJson(*itr);
    if(single_log_tree)
    {
      //ptree child;
      //child.add_child("data",*single_log_tree);
      log_tree.push_back(std::make_pair("", *single_log_tree));
    }
    else
    {
      ROS_ERROR_STREAM("failed to convert to json");
    }
  }
  pt.add_child("log",log_tree);
  return pt;
}

} // namespace scenario_logger
