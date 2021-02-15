// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SCENARIO_SEQUENCE__SEQUENCE_HPP_
#define SCENARIO_SEQUENCE__SEQUENCE_HPP_

#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <utility>

#include "scenario_expression/expression.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_sequence/event_manager.hpp"
#include "scenario_utility/scenario_utility.hpp"


namespace scenario_sequence
{

class Sequence
{
  scenario_expression::Context context_;

  const std::string name_;

  std::shared_ptr<scenario_sequence::EventManager> event_manager_;

  scenario_expression::Expression start_condition_;

  bool ignited_;

public:
  Sequence(const scenario_expression::Context &, const YAML::Node &);

  const auto & name() const noexcept
  {
    return name_;
  }

  template<typename ... Ts>
  decltype(auto) current_event_name(Ts && ... xs) const
  {
    return (*event_manager_).current_event_name(std::forward<decltype(xs)>(xs)...);
  }

  auto property() const
  {
    boost::property_tree::ptree result {};

    result.put("Name", name());
    result.put("State", boost::lexical_cast<std::string>(currently));
    result.add_child("StartConditions", start_condition_.property());
    result.add_child("Events", (*event_manager_).property());

    return result;
  }

  state_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager> &);

  state_is currently;
};

}  // namespace scenario_sequence

#endif  // SCENARIO_SEQUENCE__SEQUENCE_HPP_
