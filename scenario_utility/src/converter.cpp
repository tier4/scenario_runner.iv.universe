#include <scenario_utility/converter.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace scenario_utility
{
namespace converter
{
geometry_msgs::msg::Quaternion convert(geometry_msgs::msg::Vector3 rpy)
{
  geometry_msgs::msg::Quaternion quat;
  tf2::Quaternion q;
  q.setRPY(rpy.x, rpy.y, rpy.z);
  quat.x = q.x();
  quat.y = q.y();
  quat.z = q.z();
  quat.w = q.w();
  return quat;
}

geometry_msgs::msg::Vector3 convert(geometry_msgs::msg::Quaternion quat)
{
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  return rpy;
}
}  // namespace converter
}  // namespace scenario_utility
