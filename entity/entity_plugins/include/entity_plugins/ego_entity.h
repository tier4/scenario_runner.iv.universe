#ifndef ENTITY_PLUGINS_EGO_CAR_ENTITIY_H_INCLUDED
#define ENTITY_PLUGINS_EGO_CAR_ENTITIY_H_INCLUDED

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <scenario_entities/entity_base.h>

namespace entity_plugins
{

class EgoEntity
  : public scenario_entities::EntityBase
{
public:
  EgoEntity();

  bool init() override;

  bool configure(
    const YAML::Node&,
    const std::shared_ptr<ScenarioAPI>&)
    override;

private:
  std::string urdf_;
  std::string initial_frame_id_;

  std::shared_ptr<scenario_actions::ActionManager>         action_manager_ptr_;
  std::shared_ptr<scenario_actions::ActionManager> initial_action_manager_ptr_;
};

}  // namespace entity_plugins

#endif  // ENTITY_PLUGINS_EGO_CAR_ENTITIY_H_INCLUDED
