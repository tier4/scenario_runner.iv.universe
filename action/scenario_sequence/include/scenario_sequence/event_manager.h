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

#ifndef SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED
#define SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED

#include <yaml-cpp/yaml.h>

#include <list>
#include <memory>
#include <queue>

#include "scenario_expression/expression.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_sequence/event.h"
#include "scenario_utility/scenario_utility.hpp"


namespace scenario_sequence
{

class EventManager
{
  scenario_expression::Context context_;

  std::list<scenario_sequence::Event> events_;

  decltype(events_)::iterator cursor;

public:
  EventManager(const scenario_expression::Context&, const YAML::Node&);

  void touch() const;

  state_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);

  state_is currently;
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_EVENT_MANAGER_H_INCLUDED

