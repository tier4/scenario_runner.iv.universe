#ifndef SCENARIO_CONDITIONS_CONDITION_LOGGER_H_INCLUDED
#define SCENARIO_CONDITIONS_CONDITION_LOGGER_H_INCLUDED
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace scenario_conditions
{
// pre declaration
class ConditionManager;

class ConditionVisualizer
{
public:
  ConditionVisualizer();
  void publishMarker(ConditionManager& manager);

private:
  void addMarker(std::string name, bool result, bool is_success_condition);

  ros::Publisher pub_marker_;
  visualization_msgs::MarkerArray marker_array_;
};
}  // namespace scenario_conditions
#endif  // SCENARIO_CONDITIONS_CONDITION_LOGGER_H_INCLUDED