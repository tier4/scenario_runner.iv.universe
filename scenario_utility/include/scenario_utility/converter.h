#ifndef SCENARIO_UTILS_CONVERTER_H_INCLUDED
#define SCENARIO_UTILS_CONVERTER_H_INCLUDED

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2/transform_datatypes.h>

namespace scenario_utility
{
inline namespace converter
{

geometry_msgs::msg::Quaternion convert(geometry_msgs::msg::Vector3 rpy);
geometry_msgs::msg::Vector3 convert(geometry_msgs::msg::Quaternion quat);

}  // namespace converter
}  // namespace scenario_utility

#endif  // SCENARIO_UTILS_CONVERTER_H_INCLUDED
