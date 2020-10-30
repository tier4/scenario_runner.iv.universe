#ifndef SCENARIO_CONDITIONS_CONDITION_BASE_H_INCLUDED
#define SCENARIO_CONDITIONS_CONDITION_BASE_H_INCLUDED

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

  ConditionBase(const std::string & type, std::size_t occurrence = 0)
    : type_ { type }
    // , name_ { type + "Condition(" + std::to_string(occurrence) + ")" }
  {}

  template <typename T>
  using Comparator = std::function<bool(const T &, const T &)>;

  virtual bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) = 0;
  virtual bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) = 0;

  const std::string & getName() const noexcept { return name_; }

  double getValue() const noexcept { return value_; }

  const auto& rename(const std::string& new_name)
  {
    return name_ = new_name;
  }

  const bool getResult() const noexcept { return result_; }

  const std::string & getType() const noexcept { return type_; }

  friend std::ostream& operator <<(std::ostream& os, const ConditionBase& datum)
  {
    return os << indent
              << "{ Name: \x1b[36m" << std::quoted(datum.getName()) << "\x1b[0m"
              << ", Value: " << datum.getValue()
              << ", Result: " << (datum.getResult() ? "\x1b[32m" : "\x1b[31m") << std::boolalpha << datum.getResult() << "\x1b[0m"
              << "},\n";
  }

protected:
  std::shared_ptr<ScenarioAPI> api_ptr_;
  YAML::Node node_;

  bool configured_ = false;
  bool keep_ = false;
  bool result_ = false;

  double value_ { std::numeric_limits<double>::quiet_NaN() };

  std::string name_;
  std::string type_;
};

}  // namespace scenario_conditions

#endif  // SCENARIO_CONDITIONS_CONDITION_BASE_H_INCLUDED
