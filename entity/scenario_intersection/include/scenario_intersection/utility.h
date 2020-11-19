#ifndef SCENARIO_INTERSECTION_UTILITY_H_INCLUDED
#define SCENARIO_INTERSECTION_UTILITY_H_INCLUDED

#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/logger.hpp>

#include <yaml-cpp/yaml.h>

namespace scenario_intersection
{

template <typename T>
T convert(const std::string&);

template <typename T, typename F>
decltype(auto) if_exist(const YAML::Node& node, const std::string& key, F&& consequent, const rclcpp::Logger & logger)
{
  if (const auto& element {node[key]})
  {
    try
    {
      return consequent(element.as<T>());
    }
    catch (const YAML::BadConversion&)
    {
      RCLCPP_ERROR_STREAM(logger,
        "A bad-conversion exception occurred when parsing the key '" << key << "'. "
        "You have specified in your scenario a value of a type that is not appropriate for that key.");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger, "Missing key '" << key << "' in following tree.\n" << node);
  }
}

} // namespace scenario_intersection

#endif // SCENARIO_INTERSECTION_UTILITY_H_INCLUDED

