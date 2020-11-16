#ifndef SCENARIO_CONDITIONS_CONDITION_LOGGER_H_INCLUDED
#define SCENARIO_CONDITIONS_CONDITION_LOGGER_H_INCLUDED

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace scenario_conditions
{
// pre declaration
class ConditionManager;

class ConditionVisualizer
{
public:
  ConditionVisualizer(const rclcpp::Node::SharedPtr node);
  void publishMarker(ConditionManager& manager);

private:
  void addMarker(std::string name, bool result, bool is_success_condition);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
};
}  // namespace scenario_conditions
#endif  // SCENARIO_CONDITIONS_CONDITION_LOGGER_H_INCLUDED