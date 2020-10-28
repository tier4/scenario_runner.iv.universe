#ifndef INCLUDED_SCENARIO_UTILITY_INDENTATION_HPP
#define INCLUDED_SCENARIO_UTILITY_INDENTATION_HPP

#include <cstddef>
#include <iostream>
#include <string>

inline namespace scenario_utility
{

struct Indentation
{
  std::size_t depth { 0 };

  auto & reset()
  {
    depth = 0;
    return *this;
  }

  auto & operator++() {          ++depth; return *this; }
  auto & operator--() { depth && --depth; return *this; }

  auto operator++(int) { auto result { *this };          ++depth; return result; }
  auto operator--(int) { auto result { *this }; depth && --depth; return result; }
};

std::ostream& operator<<(std::ostream&, const Indentation&);

extern Indentation& indent;

static struct IndentationInitializer
{
  IndentationInitializer();
  ~IndentationInitializer();
} indentation_initializer;

}  // namespace scenario_utility

#endif // INCLUDED_SCENARIO_UTILITY_INDENTATION_HPP
