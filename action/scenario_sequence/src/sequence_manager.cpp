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

#include "scenario_sequence/sequence_manager.h"

namespace scenario_sequence
{

SequenceManager::SequenceManager(
  const scenario_expression::Context& context, const YAML::Node& sequences)
  : context_ { context }
  , currently { state_is::sleeping }
{
  for (const auto& each : sequences)
  {
    sequences_.emplace_back(context, each["Sequence"]);
  }

  cursor = std::begin(sequences_);
}

state_is SequenceManager::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  std::cout << "  Sequences: [" << std::endl;

  std::cout << "\x1b[2m";
  for (auto iter { std::begin(sequences_) }; iter != cursor; ++iter)
  {
    (*iter).touch();
  }
  std::cout << "\x1b[0m";

  if (cursor != std::end(sequences_))
  {
    switch (currently = (*cursor).update(context_.intersections_pointer()))
    {
    case state_is::finished:
      for (auto iter { ++cursor }; iter != std::end(sequences_); ++iter)
      {
        (*iter).touch();
      }
      break;

    default:
      for (auto iter { std::next(cursor) }; iter != std::end(sequences_); ++iter)
      {
        (*iter).touch();
      }
      break;
    }
  }
  else
  {
    currently = state_is::finished;
  }

  std::cout << "  ]," << std::endl;
}

} // namespace scenario_sequence

