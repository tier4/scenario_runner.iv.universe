#ifndef CONDITION_PLUGINS_COLLISION_BY_ENTITY_CONDITION_H_INCLUDED
#define CONDITION_PLUGINS_COLLISION_BY_ENTITY_CONDITION_H_INCLUDED

#include <scenario_conditions/condition_base.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_utility/scenario_utility.h>

namespace condition_plugins
{
class CollisionByEntityCondition : public scenario_conditions::ConditionBase
{
public:
  CollisionByEntityCondition();
  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) override;

private:
  std::string trigger_, target_entity_;
};
}  // namespace condition_plugins

#endif  // CONDITION_PLUGINS_COLLISION_BY_ENTITY_CONDITION_H_INCLUDED
