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

#ifndef SCENARIO_INTERSECTION__ARROW_HPP_
#define SCENARIO_INTERSECTION__ARROW_HPP_

#include <ostream>
#include <string>
#include <utility>

#include "scenario_intersection/utility.hpp"

namespace scenario_intersection
{

enum class Arrow : int
{
  Blank = 0,
  Left,
  LeftRight,
  Right,
  Straight,
  StraightLeft,
  StraightRight,
};

template<>
Arrow convert<Arrow>(const std::string &);

std::ostream & operator<<(std::ostream & os, const Arrow arrow);

}  // namespace scenario_intersection
#endif  // SCENARIO_INTERSECTION__ARROW_HPP_
