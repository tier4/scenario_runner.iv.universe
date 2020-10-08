#ifndef SCENARIO_ACTIONS_ENTITY_ACTION_BASE_H_INCLUDED
#define SCENARIO_ACTIONS_ENTITY_ACTION_BASE_H_INCLUDED

#include <memory>

#include <yaml-cpp/yaml.h>

#include <scenario_api/scenario_api_core.h>
#include <scenario_intersection/intersection_manager.h>

namespace scenario_actions
{

class EntityActionBase
{
public:
  virtual auto run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    -> void
    = 0;

  virtual void configure(
    const YAML::Node & node, const std::vector<std::string> actors,
    const std::shared_ptr<ScenarioAPI> & api_ptr)
  {
    node_ = node;
    actors_ = actors;
    api_ptr_ = api_ptr;
  }

  EntityActionBase() = default;

  EntityActionBase(const std::string & type)
    : type_ { type }
    , name_ { type + "Action<" + boost::lexical_cast<std::string>(this) + ">" }
  {}

protected:
  std::string type_, name_;

  YAML::Node node_;

  std::vector<std::string> actors_;

  std::shared_ptr<ScenarioAPI> api_ptr_;
};

}  // namespace scenario_actions

#endif  // SCENARIO_ACTIONS_ENTITY_ACTION_BASE_H_INCLUDED
