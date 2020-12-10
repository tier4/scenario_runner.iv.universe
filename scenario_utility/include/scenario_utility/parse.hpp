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

#ifndef SCENARIO_UTILITY__PARSE_HPP_
#define SCENARIO_UTILITY__PARSE_HPP_

#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "scenario_logger/logger.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"
#include "boost/optional.hpp"

inline namespace scenario_utility
{
inline namespace parse
{
std::vector<std::string> split(std::string s);

template<typename T>
T read_as(const YAML::Node &);

#define READ_AS_SPECIALIZED_SIGNATURE(TYPENAME) \
  template<> \
  TYPENAME read_as<TYPENAME>(const YAML::Node & node)

READ_AS_SPECIALIZED_SIGNATURE(geometry_msgs::msg::Point);
READ_AS_SPECIALIZED_SIGNATURE(geometry_msgs::msg::Quaternion);
READ_AS_SPECIALIZED_SIGNATURE(geometry_msgs::msg::Pose);
READ_AS_SPECIALIZED_SIGNATURE(geometry_msgs::msg::PoseStamped);

template<typename T>
T read_essential(const YAML::Node & node, const std::string & key)
{
  if (const auto x {node[key]}) {
    try {
      return x.as<T>();
    } catch (...) {
      SCENARIO_ERROR_THROW(
        CATEGORY(),
        "syntax-error: unexpected type appeared.\n\n" << node[key] << "\n");
    }
  } else {
    SCENARIO_ERROR_THROW(
      CATEGORY(),
      "syntax-error: missing essential clause " << key << ".\n\n" << node << "\n");
  }
}

#define READ_ESSENTIAL_SPECIALIZED_SIGNATURE(TYPENAME) \
  template<> \
  TYPENAME read_essential<TYPENAME>( \
    const YAML::Node & node, const std::string & key)

READ_ESSENTIAL_SPECIALIZED_SIGNATURE(geometry_msgs::msg::Point);
READ_ESSENTIAL_SPECIALIZED_SIGNATURE(geometry_msgs::msg::Quaternion);
READ_ESSENTIAL_SPECIALIZED_SIGNATURE(geometry_msgs::msg::Pose);
READ_ESSENTIAL_SPECIALIZED_SIGNATURE(geometry_msgs::msg::PoseStamped);

template<typename T>
T read_optional(
  const YAML::Node & node, const std::string & key, const T & value = {})
{
  if (const auto x {node[key]}) {
    try {
      return x.as<T>();
    } catch (...) {
      SCENARIO_ERROR_THROW(
        CATEGORY(),
        "syntax-error: unmatched type.\n\n" << node[key] << "\n");
    }
  } else {
    SCENARIO_WARN_STREAM(
      CATEGORY(),
      "syntax-warning: missing optional clause " << key <<
        ". Use default value " << value << "\n\n" << node << "\n");
    return value;
  }
}

template<typename T, typename F,
  typename =
  typename std::enable_if<
    std::is_function<
      typename std::decay<F>::type
    >::value
  >::type>
T read_optional(const YAML::Node & node, const std::string & key, F && f)
{
  if (const auto x {node[key]}) {
    try {
      return x.as<T>();
    } catch (...) {
      SCENARIO_ERROR_THROW(
        CATEGORY(),
        "syntax-error: unmatched type.\n\n" << node[key] << "\n");
    }
  } else {
    SCENARIO_WARN_STREAM(
      CATEGORY(),
      "syntax-warning: missing optional clause " << key << ". Use default value.\n\n" << node <<
        "\n");
    return f();
  }
}

template<typename F>
decltype(auto) call_with_essential(
  const YAML::Node & node, const std::string & key, F && f)
{
  if (const auto x {node[key]}) {
    return f(x);
  } else {
    SCENARIO_ERROR_THROW(
      CATEGORY(),
      "syntax-error: missing essential clause " << key << ".\n\n" << node << "\n");
  }
}

template<typename F>
void call_with_optional(
  const YAML::Node & node, const std::string & key, F && f)
{
  if (const auto x {node[key]}) {
    return f(x);
  } else {
    SCENARIO_WARN_STREAM(
      CATEGORY(),
      "syntax-warning: missing optional clause " << key << ".\n\n" << node << "\n");
  }
}

template<typename T>
bool parseRule(std::string rule_string, std::function<bool(const T &, const T &)> & compare)
{
  if (rule_string == "Equal" || rule_string == "eq" || rule_string == "==") {
    compare = std::equal_to<T>();
  } else if (rule_string == "NotEqual" || rule_string == "neq" || rule_string == "!=") {
    compare = std::not_equal_to<T>();
  } else if (rule_string == "GreaterThan" || rule_string == "gt" || rule_string == ">") {
    compare = std::greater<T>();
  } else if (rule_string == "GreaterEqual" || rule_string == "ge" || rule_string == ">=") {
    compare = std::greater_equal<T>();
  } else if (rule_string == "LessThan" || rule_string == "lt" || rule_string == "<") {
    compare = std::less<T>();
  } else if (rule_string == "LessEqual" || rule_string == "le" || rule_string == "<=") {
    compare = std::less_equal<T>();
  } else {
    return false;
  }
  return true;
}
}  // namespace parse
}  // namespace scenario_utility

#endif  // SCENARIO_UTILITY__PARSE_HPP_
