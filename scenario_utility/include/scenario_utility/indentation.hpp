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

#ifndef SCENARIO_UTILITY__INDENTATION_HPP_
#define SCENARIO_UTILITY__INDENTATION_HPP_

#include <cstddef>
#include <iostream>
#include <string>

inline namespace scenario_utility
{
struct Indentation
{
  std::size_t depth {0};

  auto & reset()
  {
    depth = 0;
    return *this;
  }

  auto & operator++() {++depth; return *this;}
  auto & operator--() {depth && --depth; return *this;}

  auto operator++(int) {auto result {*this};          ++depth; return result;}
  auto operator--(int) {auto result {*this}; depth && --depth; return result;}
};

std::ostream & operator<<(std::ostream &, const Indentation &);

extern Indentation & indent;

static struct IndentationInitializer
{
  IndentationInitializer();
  ~IndentationInitializer();
} indentation_initializer;
}  // namespace scenario_utility
#endif  // SCENARIO_UTILITY__INDENTATION_HPP_
