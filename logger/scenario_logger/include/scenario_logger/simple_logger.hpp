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

#ifndef SCENARIO_LOGGER__SIMPLE_LOGGER_HPP_
#define SCENARIO_LOGGER__SIMPLE_LOGGER_HPP_

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace scenario_logger
{
inline namespace simple
{
decltype(auto) elog(std::ostream & os)
{
  return os << "\" }" << std::endl;
}

struct SimpleLogger
  : public std::ofstream
{
  struct Header
  {
    const std::string level;

    explicit Header(const std::string & level)
    : level{level}
    {}

    friend decltype(auto) operator<<(std::ostream & os, const Header & header)
    {
      return os << "{ \"level\": " << std::quoted(header.level) << ", \"message\": \"";
    }
  };

  decltype(auto) info() {return (*this) << Header("info");}
  decltype(auto) error() {return (*this) << Header("error");}
};

extern SimpleLogger & slog;

static struct SimpleLoggerInitializer
{
  SimpleLoggerInitializer();
  ~SimpleLoggerInitializer();
} initializer;
}  // namespace simple
}  // namespace scenario_logger


#define LOG_SIMPLE(...)                                                     /**/ \
  do {                                                                      /**/ \
    using scenario_logger::slog;                                            /**/ \
    using scenario_logger::elog;                                            /**/ \
                                                                            /**/ \
    slog.__VA_ARGS__ << elog;                                               /**/ \
  }                                                                         /**/ \
  while (false)

#define LOG_SIMPLE_ONCE(...)                                                /**/ \
  do {                                                                      /**/ \
    using scenario_logger::slog;                                            /**/ \
    using scenario_logger::elog;                                            /**/ \
                                                                            /**/ \
    static auto logged {false};                                             /**/ \
                                                                            /**/ \
    if (!logged) {                                                          /**/ \
      slog.__VA_ARGS__ << elog;                                             /**/ \
      logged = true;                                                        /**/ \
    }                                                                       /**/ \
  }                                                                         /**/ \
  while (false)

#define LOG_TOGGLE(TITLE, VARIABLE)                                         /**/ \
  do {                                                                      /**/ \
    using scenario_logger::slog;                                            /**/ \
    using scenario_logger::elog;                                            /**/ \
                                                                            /**/ \
    static auto previous {                                                  /**/ \
      ((slog.info() << TITLE ": " << VARIABLE << elog), VARIABLE)           /**/ \
    };                                                                      /**/ \
                                                                            /**/ \
    if (previous != VARIABLE) {                                             /**/ \
      slog.info() << TITLE ": " << previous << " => " << VARIABLE << elog;  /**/ \
      previous = VARIABLE;                                                  /**/ \
    }                                                                       /**/ \
  }                                                                         /**/ \
  while (false)  // NOLINT

#endif  // SCENARIO_LOGGER__SIMPLE_LOGGER_HPP_
