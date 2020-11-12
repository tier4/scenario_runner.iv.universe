#ifndef SCENARIO_CONDITIONS_CONDITION_BASE_H_INCLUDED
#define SCENARIO_CONDITIONS_CONDITION_BASE_H_INCLUDED

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <yaml-cpp/yaml.h>

#include <scenario_api/scenario_api_core.h>
#include <scenario_intersection/intersection_manager.h>
#include <limits>
#include <scenario_utility/indentation.hpp>

namespace scenario_conditions
{
class ConditionBase
{
public:
  ConditionBase() = default;

  ConditionBase(const std::string & type)
    : type_ { type }
    , name_ {}
  {}

  template <typename T>
  using Comparator = std::function<bool(const T &, const T &)>;

  virtual bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) = 0;
  virtual bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) = 0;

  const std::string & getName() const noexcept { return name_; }

  const std::string& description() const noexcept { return description_; }

  const auto& rename(const std::string& new_name)
  {
    return name_ = new_name;
  }

  const bool getResult() const noexcept { return result_; }

  const std::string & getType() const noexcept { return type_; }

  auto property() const
  {
    boost::property_tree::ptree result {};

    result.put("Name", getName());
    result.put("Value", description());
    result.put("Result", getResult());

    return result;
  }

protected:
  std::shared_ptr<ScenarioAPI> api_ptr_;
  YAML::Node node_;

  bool configured_ = false;
  bool keep_ = false;
  bool result_ = false;

  std::string description_;

  std::string name_;
  std::string type_;
};

}  // namespace scenario_conditions

#endif  // SCENARIO_CONDITIONS_CONDITION_BASE_H_INCLUDED
