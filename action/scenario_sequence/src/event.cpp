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

#include "scenario_sequence/event.h"

namespace scenario_sequence
{

Event::Event(
  const scenario_expression::Context& context,
  const YAML::Node& event_definition)
  : context_ { context }
  , name_ { event_definition["Name"].as<std::string>() }
  , ignited_ { false }
  , currently { state_is::sleeping }
{
  for (const auto& each : event_definition["Actors"])
  {
    actors_.push_back(each.as<std::string>());
  }

  action_manager_ =
    std::make_shared<scenario_actions::ActionManager>(
      event_definition["Actions"], actors_, context.api_pointer());

  if (const auto condition { event_definition["Condition"] })
  {
    condition_ = scenario_expression::read(context_, condition);
  }
  else // NOTE: If Condition unspecified, the sequence starts unconditionally.
  {
    ignited_ = true;
  }
}

void Event::touch()
{
  std::cout << "        {\n";
  std::cout << "          Name: " << name_ << ",\n";
  std::cout << "          Conditions: [\n";
  std::cout << "            TODO,\n";
  std::cout << "          ],\n";
  std::cout << "          State: " << currently << ",\n";
  std::cout << "        },\n";
}

state_is Event::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  std::cout << "        {\n";
  std::cout << "          Name: " << name_ << ",\n";

  std::cout << "          Conditions: [\n";
  ignited_ = condition_.evaluate(context_);
  std::cout << "          ],\n";

  if (ignited_)
  {
    (*action_manager_).run(context_.intersections_pointer());
    currently = state_is::finished;
  }
  else
  {
    currently = state_is::running;
  }

  std::cout << "          State: " << currently << ",\n";
  std::cout << "        },\n";

  return currently;
}

std::ostream& operator <<(std::ostream& os, const state_is& currently)
{
  switch (currently)
  {
  case state_is::sleeping:
    return os << std::quoted("NotRunning");

  case state_is::running:
    return os << std::quoted("Running");

  case state_is::finished:
    return os << std::quoted("Finished");
  }
}

} // namespace scenario_sequence

