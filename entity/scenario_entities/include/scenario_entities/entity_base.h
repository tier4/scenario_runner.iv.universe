#ifndef SCENARIO_ENTITIES_ENTITIY_BASE_H_INCLUDED
#define SCENARIO_ENTITIES_ENTITIY_BASE_H_INCLUDED

#include <memory>
#include <sstream>
#include <string>

#include <boost/algorithm/string.hpp>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <scenario_actions/action_manager.h>
#include <scenario_api/scenario_api_core.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_logger/logger.h>
#include <scenario_utility/scenario_utility.h>

namespace scenario_entities
{

class EntityBase
{
public:
  EntityBase(const std::string& type)
    : type_ {type}
  {}

  const std::string& getName() const
  {
    return name_;
  };

  const std::string& getType() const
  {
    return type_;
  };

  virtual bool configure(
    const YAML::Node& entity,
    const std::shared_ptr<ScenarioAPI>& api);

  virtual bool setStory(const YAML::Node& story);

  virtual bool init();

  virtual simulation_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager> &);

protected:
  std::string name_;
  std::string type_;

  YAML::Node act_;
  YAML::Node end_condition_;
  YAML::Node init_;
  YAML::Node init_entity_;

  std::shared_ptr<ScenarioAPI> api_;

  std::shared_ptr<scenario_actions::ActionManager> action_manager_;

  geometry_msgs::PoseStamped pose_stamped_;

  float speed_;
};

}  // namespace scenario_entities

#endif  // SCENARIO_ENTITIES_ENTITIY_BASE_H_INCLUDED

