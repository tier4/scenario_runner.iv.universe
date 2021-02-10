#include <scenario_logger/simple_logger.hpp>

namespace scenario_logger { inline namespace simple
{

static std::size_t schwarz_counter { 0 };

static typename std::aligned_storage<
  sizeof(SimpleLogger), alignof(SimpleLogger)
>::type memory;

SimpleLogger& slog { reinterpret_cast<SimpleLogger&>(memory) };

SimpleLoggerInitializer::SimpleLoggerInitializer()
{
  if (schwarz_counter++ == 0)
  {
    new (&slog) SimpleLogger();
  }
}

SimpleLoggerInitializer::~SimpleLoggerInitializer()
{
  if (--schwarz_counter == 0)
  {
    slog.~SimpleLogger();
  }
}

}} // namespace scenario_logger
