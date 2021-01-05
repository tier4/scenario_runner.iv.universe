#ifndef SCENARIO_UTILS_CONVERTER_H_INCLUDED
#define SCENARIO_UTILS_CONVERTER_H_INCLUDED

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/transform_datatypes.h>

inline namespace scenario_utility
{
inline namespace converter
{

geometry_msgs::Quaternion convert(geometry_msgs::Vector3 rpy);
geometry_msgs::Vector3 convert(geometry_msgs::Quaternion quat);

}  // namespace converter
}  // namespace scenario_utility

#endif  // SCENARIO_UTILS_CONVERTER_H_INCLUDED
