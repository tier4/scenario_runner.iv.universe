#ifndef SCENARIO_UTILS_PARSE_H_INCLUDED
#define SCENARIO_UTILS_PARSE_H_INCLUDED

#include <boost/optional.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <scenario_logger/logger.h>
#include <sstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

inline namespace scenario_utility
{
inline namespace parse
{
std::vector<std::string> split(std::string s);

template <typename T>
T read_as(const YAML::Node&);

#define READ_AS_SPECIALIZED_SIGNATURE(TYPENAME)                                \
  template <>                                                                  \
  TYPENAME read_as<TYPENAME>(const YAML::Node& node)

READ_AS_SPECIALIZED_SIGNATURE(geometry_msgs::Point);
READ_AS_SPECIALIZED_SIGNATURE(geometry_msgs::Quaternion);
READ_AS_SPECIALIZED_SIGNATURE(geometry_msgs::Pose);
READ_AS_SPECIALIZED_SIGNATURE(geometry_msgs::PoseStamped);

template <typename T>
T read_essential(const YAML::Node& node, const std::string& key)
{
  if (const auto x { node[key] })
  {
    try
    {
      return x.as<T>();
    }
    catch (...)
    {
      SCENARIO_ERROR_THROW(CATEGORY(),
        "syntax-error: unexpected type appeared.\n\n" << node[key] << "\n");
    }
  }
  else
  {
    SCENARIO_ERROR_THROW(CATEGORY(),
      "syntax-error: missing essential clause " << key << ".\n\n" << node << "\n");
  }
}

#define READ_ESSENTIAL_SPECIALIZED_SIGNATURE(TYPENAME)                         \
  template <>                                                                  \
  TYPENAME read_essential<TYPENAME>(                                           \
    const YAML::Node& node, const std::string& key)

READ_ESSENTIAL_SPECIALIZED_SIGNATURE(geometry_msgs::Point);
READ_ESSENTIAL_SPECIALIZED_SIGNATURE(geometry_msgs::Quaternion);
READ_ESSENTIAL_SPECIALIZED_SIGNATURE(geometry_msgs::Pose);
READ_ESSENTIAL_SPECIALIZED_SIGNATURE(geometry_msgs::PoseStamped);

template <typename T>
T read_optional(
  const YAML::Node& node, const std::string& key, const T& value = {})
{
  if (const auto x { node[key] })
  {
    try
    {
      return x.as<T>();
    }
    catch (...)
    {
      SCENARIO_ERROR_THROW(CATEGORY(),
        "syntax-error: unmatched type.\n\n" << node[key] << "\n");
    }
  }
  else
  {
    SCENARIO_WARN_STREAM(CATEGORY(),
      "syntax-warning: missing optional clause " << key << ". Use default value " << value << "\n\n" << node << "\n");
    return value;
  }
}

template <typename T, typename F,
          typename =
            typename std::enable_if<
              std::is_function<
                typename std::decay<F>::type
              >::value
            >::type>
T read_optional(const YAML::Node& node, const std::string& key, F&& f)
{
  if (const auto x { node[key] })
  {
    try
    {
      return x.as<T>();
    }
    catch (...)
    {
      SCENARIO_ERROR_THROW(CATEGORY(),
        "syntax-error: unmatched type.\n\n" << node[key] << "\n");
    }
  }
  else
  {
    SCENARIO_WARN_STREAM(CATEGORY(),
      "syntax-warning: missing optional clause " << key << ". Use default value.\n\n" << node << "\n");
    return f();
  }
}

template <typename F>
decltype(auto) call_with_essential(
  const YAML::Node& node, const std::string& key, F&& f)
{
  if (const auto x { node[key] })
  {
    return f(x);
  }
  else
  {
    SCENARIO_ERROR_THROW(CATEGORY(),
      "syntax-error: missing essential clause " << key << ".\n\n" << node << "\n");
  }
}

template <typename F>
void call_with_optional(
  const YAML::Node& node, const std::string& key, F&& f)
{
  if (const auto x { node[key] })
  {
    return f(x);
  }
  else
  {
    SCENARIO_WARN_STREAM(CATEGORY(),
      "syntax-warning: missing optional clause " << key << ".\n\n" << node << "\n");
  }
}

template <typename T>
bool parseRule(std::string rule_string, std::function<bool(const T &, const T &)> & compare)
{
  if (rule_string == "Equal" or rule_string == "eq" or rule_string == "==") {
    compare = std::equal_to<T>();
  } else if (rule_string == "NotEqual" or rule_string == "neq" or rule_string == "!=") {
    compare = std::not_equal_to<T>();
  } else if (rule_string == "GreaterThan" or rule_string == "gt" or rule_string == ">") {
    compare = std::greater<T>();
  } else if (rule_string == "GreaterEqual" or rule_string == "ge" or rule_string == ">=") {
    compare = std::greater_equal<T>();
  } else if (rule_string == "LessThan" or rule_string == "lt" or rule_string == "<") {
    compare = std::less<T>();
  } else if (rule_string == "LessEqual" or rule_string == "le" or rule_string == "<=") {
    compare = std::less_equal<T>();
  } else {
    return false;
  }
  return true;
}
}  // namespace parse
}  // namespace scenario_utility

#endif  // SCENARIO_UTILS_PARSE_H_INCLUDED
