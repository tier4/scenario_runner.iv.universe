#ifndef ACION_PLUGINS_FOLLOW_ROUTE_ACTION_H_INCLUDED
#define ACION_PLUGINS_FOLLOW_ROUTE_ACTION_H_INCLUDED

#include <scenario_actions/entity_action_base.h>
#include <scenario_intersection/intersection_manager.h>
#include <yaml-cpp/node/node.h>
#include <geometry_msgs/msg/pose.hpp>
namespace action_plugins
{

class FollowRouteAction
  : public scenario_actions::EntityActionBase
{
  geometry_msgs::msg::Pose goal_;

  std::string shift_;

public:
  FollowRouteAction();

  void configure(
    const YAML::Node& node,
    const std::vector<std::string> actors,
    const std::shared_ptr<ScenarioAPI>& api_ptr) override;

  auto run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    -> void override;
};

}  // namespace action_plugins

#endif  // ACION_PLUGINS_FOLLOW_ROUTE_ACTION_H_INCLUDED
