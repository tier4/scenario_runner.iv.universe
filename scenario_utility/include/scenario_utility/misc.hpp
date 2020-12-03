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

#ifndef SCENARIO_UTILS_MISC_H_INCLUDED
#define SCENARIO_UTILS_MISC_H_INCLUDED

#include <ostream>

// namespace scenario_utility
// {
// inline namespace misc
// {

enum class simulation_is : int
{
  // XXX DON'T SORT THIS!!!
  failed = -1,
  ongoing,
  succeeded,
};

std::ostream& operator<<(std::ostream&, const simulation_is&);

constexpr simulation_is operator &&(const simulation_is& lhs,
                                    const simulation_is& rhs)
{
  using comparable = std::underlying_type<simulation_is>::type;
  return
    static_cast<simulation_is>(
      std::min<comparable>(
        static_cast<comparable>(lhs),
        static_cast<comparable>(rhs)));
}

constexpr simulation_is operator ||(const simulation_is& lhs,
                                    const simulation_is& rhs)
{
  using comparable = std::underlying_type<simulation_is>::type;
  return
    static_cast<simulation_is>(
      std::max<comparable>(
        static_cast<comparable>(lhs),
        static_cast<comparable>(rhs)));
}


static_assert( ( simulation_is::succeeded && simulation_is::succeeded ) == simulation_is::succeeded, "");
static_assert( ( simulation_is::succeeded && simulation_is::ongoing   ) == simulation_is::ongoing  , "");
static_assert( ( simulation_is::succeeded && simulation_is::failed    ) == simulation_is::failed   , "");

static_assert( ( simulation_is::ongoing   && simulation_is::succeeded ) == simulation_is::ongoing  , "");
static_assert( ( simulation_is::ongoing   && simulation_is::ongoing   ) == simulation_is::ongoing  , "");
static_assert( ( simulation_is::ongoing   && simulation_is::failed    ) == simulation_is::failed   , "");

static_assert( ( simulation_is::failed    && simulation_is::succeeded ) == simulation_is::failed   , "");
static_assert( ( simulation_is::failed    && simulation_is::ongoing   ) == simulation_is::failed   , "");
static_assert( ( simulation_is::failed    && simulation_is::failed    ) == simulation_is::failed   , "");


static_assert( ( simulation_is::succeeded || simulation_is::succeeded ) == simulation_is::succeeded, "");
static_assert( ( simulation_is::succeeded || simulation_is::ongoing   ) == simulation_is::succeeded, "");
static_assert( ( simulation_is::succeeded || simulation_is::failed    ) == simulation_is::succeeded, "");

static_assert( ( simulation_is::ongoing   || simulation_is::succeeded ) == simulation_is::succeeded, "");
static_assert( ( simulation_is::ongoing   || simulation_is::ongoing   ) == simulation_is::ongoing  , "");
static_assert( ( simulation_is::ongoing   || simulation_is::failed    ) == simulation_is::ongoing  , "");

static_assert( ( simulation_is::failed    || simulation_is::succeeded ) == simulation_is::succeeded, "");
static_assert( ( simulation_is::failed    || simulation_is::ongoing   ) == simulation_is::ongoing  , "");
static_assert( ( simulation_is::failed    || simulation_is::failed    ) == simulation_is::failed   , "");

// }  inline namespace misc
// }  namespace scenario_utility

#endif  // SCENARIO_UTILS_MISC_H_INCLUDED

