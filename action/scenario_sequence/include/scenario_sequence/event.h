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

#ifndef SCENARIO_SEQUENCE_EVENT_H_INCLUDED
#define SCENARIO_SEQUENCE_EVENT_H_INCLUDED


#include "scenario_actions/action_manager.hpp"
#include "scenario_expression/expression.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_utility/scenario_utility.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

namespace scenario_sequence
{

enum class state_is
{
  sleeping, running, finished,
};

std::ostream& operator <<(std::ostream&, const state_is&);

class Event
{
  scenario_expression::Context context_;

  const std::string name_;

  std::vector<std::string> actors_;

  std::shared_ptr<scenario_actions::ActionManager> action_manager_;

  scenario_expression::Expression condition_;

  bool ignited_;

public:
  Event(const scenario_expression::Context&, const YAML::Node&);

  state_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);

  void dummy();

  state_is currently;
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_EVENT_H_INCLUDED

