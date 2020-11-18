#ifndef SCENARIO_CONDITIONS_CONDITION_BASE_H_INCLUDED
#define SCENARIO_CONDITIONS_CONDITION_BASE_H_INCLUDED

#include <yaml-cpp/yaml.h>

#include <boost/lexical_cast.hpp>

#include <scenario_api/scenario_api_core.h>
#include <scenario_intersection/intersection_manager.h>

namespace scenario_conditions
{
class ConditionBase
{
public:
  ConditionBase() = default;

  ConditionBase(const std::string & type)
    : type_ { type }
    , name_ { type + "Condition<" + boost::lexical_cast<std::string>(this) + ">" }
  {}

  template <typename T>
  using Comparator = std::function<bool(const T &, const T &)>;

  virtual bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) = 0;
  virtual bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) = 0;

  const std::string & getName() const noexcept { return name_; }

  bool getResult() const noexcept { return result_; }

  const std::string & getType() const noexcept { return type_; }

protected:
  std::shared_ptr<ScenarioAPI> api_ptr_;
  YAML::Node node_;

  bool configured_ = false;
  bool keep_ = false;
  bool result_ = false;

  std::string type_;
  std::string name_;
};

}  // namespace scenario_conditions

#endif  // SCENARIO_CONDITIONS_CONDITION_BASE_H_INCLUDED
