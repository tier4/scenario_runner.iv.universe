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

#ifndef SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED
#define SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED

#include <list>

#include "scenario_expression/expression.hpp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_sequence/sequence.h"
#include "scenario_utility/scenario_utility.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <queue>

namespace scenario_sequence
{

class SequenceManager
{
  std::list<scenario_sequence::Sequence> sequences_;

  scenario_expression::Context context_;

  // NOTE: Adding, removing and moving the elements within the list or across
  // several lists does not invalidate the iterators or references.
  decltype(sequences_)::iterator cursor;

public:
  SequenceManager(const scenario_expression::Context&, const YAML::Node&);

  const auto& current_sequence_name() const
  {
    if (cursor != std::end(sequences_))
    {
      return (*cursor).name();
    }
    else
    {
      static const std::string it { "" };
      return it;
    }
  }

  const auto& current_event_name() const
  {
    if (cursor != std::end(sequences_))
    {
      return (*cursor).current_event_name();
    }
    else
    {
      static const std::string it { "" };
      return it;
    }
  }

  auto property() const
  {
    boost::property_tree::ptree result {};

    for (const auto& each : sequences_)
    {
      result.push_back(std::make_pair("", each.property()));
    }

    return result;
  }

  state_is update(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&);

  state_is currently;
};

} // namespace scenario_sequence

#endif // SCENARIO_SEQUENCE_SEQUENCE_MANAGER_H_INCLUDED

