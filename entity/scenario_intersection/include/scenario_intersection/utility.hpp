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

#ifndef SCENARIO_INTERSECTION__UTILITY_HPP_
#define SCENARIO_INTERSECTION__UTILITY_HPP_

#include <yaml-cpp/yaml.h>

#include <string>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp/logger.hpp"


namespace scenario_intersection
{

template<typename T>
T convert(const std::string &);

template<typename T, typename F>
decltype(auto) if_exist(
  const YAML::Node & node, const std::string & key, F && consequent,
  const rclcpp::Logger & logger)
{
  if (const auto & element {node[key]}) {
    try {
      return consequent(element.as<T>());
    } catch (const YAML::BadConversion &) {
      RCLCPP_ERROR_STREAM(
        logger,
        "A bad-conversion exception occurred when parsing the key '" << key << "'. "
          "You have specified in your scenario a value of a type that is not appropriate for that key.");  // NOLINT
    }
  } else {
    RCLCPP_ERROR_STREAM(logger, "Missing key '" << key << "' in following tree.\n" << node);
  }
}

}  // namespace scenario_intersection
#endif  // SCENARIO_INTERSECTION__UTILITY_HPP_
