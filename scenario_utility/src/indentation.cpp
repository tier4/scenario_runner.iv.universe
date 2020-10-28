#include <scenario_utility/indentation.hpp>

inline namespace scenario_utility
{

static auto schwarz_counter { 0 };

static typename std::aligned_storage<sizeof(Indentation), alignof(Indentation)>::type memory;

Indentation& indent { reinterpret_cast<Indentation&>(memory) };

std::ostream& operator <<(std::ostream& os, const Indentation& datum)
{
  return os << std::string(datum.depth * 2, ' ');
}

IndentationInitializer::IndentationInitializer()
{
  if (schwarz_counter++ == 0)
  {
    new (&indent) Indentation();
  }
}

IndentationInitializer::~IndentationInitializer()
{
  if (--schwarz_counter == 0)
  {
    indent.~Indentation();
  }
}

} // namespace scenario_utility
