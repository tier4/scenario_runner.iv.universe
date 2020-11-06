#ifndef INCLUDED_SCENARIO_LOGGER_SIMPLE_LOGGER_HPP
#define INCLUDED_SCENARIO_LOGGER_SIMPLE_LOGGER_HPP

#include <fstream>
#include <iostream>

namespace scenario_logger { inline namespace simple
{
  struct SimpleLogger
    : public std::ofstream
  {
  };

  extern SimpleLogger& slog;

  static struct SimpleLoggerInitializer
  {
    SimpleLoggerInitializer();
    ~SimpleLoggerInitializer();
  } initializer;
}} // namespace scenario_logger

#endif // INCLUDED_SCENARIO_LOGGER_SIMPLE_LOGGER_HPP
