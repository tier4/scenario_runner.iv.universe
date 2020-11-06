#ifndef INCLUDED_SCENARIO_LOGGER_SIMPLE_LOGGER_HPP
#define INCLUDED_SCENARIO_LOGGER_SIMPLE_LOGGER_HPP

#include <fstream>
#include <iomanip>
#include <iostream>

namespace scenario_logger { inline namespace simple
{
  decltype(auto) endlog(std::ostream& os)
  {
    return os << "\" }," << std::endl;
  }

  struct SimpleLogger
    : public std::ofstream
  {
    struct Header
    {
      const std::string level;

      explicit Header(const std::string& level)
        : level { level }
      {}

      friend decltype(auto) operator <<(std::ostream& os, const Header& header)
      {
        return os << "{ \"level\": " << std::quoted(header.level) << ", \"message\": \"";
      }
    };

    decltype(auto) info()  { return (*this) << Header("info"); }
    decltype(auto) error() { return (*this) << Header("error"); }
  };

  extern SimpleLogger& slog;

  static struct SimpleLoggerInitializer
  {
    SimpleLoggerInitializer();
    ~SimpleLoggerInitializer();
  } initializer;
}} // namespace scenario_logger

#endif // INCLUDED_SCENARIO_LOGGER_SIMPLE_LOGGER_HPP
