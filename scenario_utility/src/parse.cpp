#include "scenario_utility/parse.h"

inline namespace scenario_utility
{
inline namespace parse
{

#define DEFINE_READ_ESSENTIAL_SPECIALIZATION(TYPENAME)                         \
  READ_ESSENTIAL_SPECIALIZED_SIGNATURE(TYPENAME)                               \
  {                                                                            \
    return call_with_essential(node, key, read_as<TYPENAME>);                  \
  }                                                                            \
  static_assert(true, "semicolon required after this macro")

template <>
geometry_msgs::Point read_as<geometry_msgs::Point>(const YAML::Node& node)
{
  geometry_msgs::Point point {};

  point.x = read_essential<float>(node, "X");
  point.y = read_essential<float>(node, "Y");
  point.z = read_essential<float>(node, "Z");

  return point;
}

DEFINE_READ_ESSENTIAL_SPECIALIZATION(geometry_msgs::Point);

template <>
geometry_msgs::Quaternion read_as<geometry_msgs::Quaternion>(const YAML::Node& node)
{
  geometry_msgs::Quaternion quaternion {};

  quaternion.x = read_essential<float>(node, "X");
  quaternion.y = read_essential<float>(node, "Y");
  quaternion.z = read_essential<float>(node, "Z");
  quaternion.w = read_essential<float>(node, "W");

  return quaternion;
}

DEFINE_READ_ESSENTIAL_SPECIALIZATION(geometry_msgs::Quaternion);

template <>
geometry_msgs::Pose read_as<geometry_msgs::Pose>(const YAML::Node& node)
{
  geometry_msgs::Pose pose {};

  pose.position = read_essential<geometry_msgs::Point>(node, "Position");
  pose.orientation = read_essential<geometry_msgs::Quaternion>(node, "Orientation");

  return pose;
}

DEFINE_READ_ESSENTIAL_SPECIALIZATION(geometry_msgs::Pose);

template <>
geometry_msgs::PoseStamped read_as<geometry_msgs::PoseStamped>(const YAML::Node& node)
{
  geometry_msgs::PoseStamped pose_stamped {};

  pose_stamped.header.frame_id = read_optional<std::string>(node, "FrameId", "/map");
  pose_stamped.header.stamp = ros::Time::now();
  // pose_stamped.pose = read_essential<geometry_msgs::Pose>(node, "Pose");
  pose_stamped.pose.position = read_essential<geometry_msgs::Point>(node, "Position");
  pose_stamped.pose.orientation = read_essential<geometry_msgs::Quaternion>(node, "Orientation");

  return pose_stamped;
}

DEFINE_READ_ESSENTIAL_SPECIALIZATION(geometry_msgs::PoseStamped);

std::vector<std::string> split(std::string s)
{
  std::vector<std::string> v;
  std::stringstream ss{ s };
  std::string buf;
  while (std::getline(ss, buf, ','))
  {
    v.push_back(buf);
  }
  return v;
}

}  // namespace parse
}  // namespace scenario_utility
