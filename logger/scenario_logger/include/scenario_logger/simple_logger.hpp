#ifndef INCLUDED_SCENARIO_LOGGER_SIMPLE_LOGGER_HPP
#define INCLUDED_SCENARIO_LOGGER_SIMPLE_LOGGER_HPP

#include <fstream>
#include <iomanip>
#include <iostream>

namespace scenario_logger { inline namespace simple
{
  decltype(auto) elog(std::ostream& os)
  {
    return os << "\" }" << std::endl;
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


#define LOG_SIMPLE(...)                                                        \
do                                                                             \
{                                                                              \
  using scenario_logger::slog;                                                 \
  using scenario_logger::elog;                                                 \
                                                                               \
  slog. __VA_ARGS__ << elog;                                                   \
}                                                                              \
while (false)


#define LOG_SIMPLE_ONCE(...)                                                   \
do                                                                             \
{                                                                              \
  using scenario_logger::slog;                                                 \
  using scenario_logger::elog;                                                 \
                                                                               \
  static auto logged { false };                                                \
                                                                               \
  if (not logged)                                                              \
  {                                                                            \
    slog. __VA_ARGS__ << elog;                                                 \
    logged = true;                                                             \
  }                                                                            \
}                                                                              \
while (false)


#define LOG_TOGGLE(TITLE, VARIABLE)                                            \
do                                                                             \
{                                                                              \
  using scenario_logger::slog;                                                 \
  using scenario_logger::elog;                                                 \
                                                                               \
  static auto previous {                                                       \
    ((slog.info() << TITLE ": " << VARIABLE << elog), VARIABLE)                \
  };                                                                           \
                                                                               \
  if (previous != VARIABLE)                                                    \
  {                                                                            \
    slog.info() << TITLE ": " << previous << " => " << VARIABLE << elog;       \
    previous = VARIABLE;                                                       \
  }                                                                            \
}                                                                              \
while (false)

#endif // INCLUDED_SCENARIO_LOGGER_SIMPLE_LOGGER_HPP
