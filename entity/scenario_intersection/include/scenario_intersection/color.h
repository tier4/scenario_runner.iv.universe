#ifndef INCLUDED_SCENARIO_INTERSECTION_COLOR_H
#define INCLUDED_SCENARIO_INTERSECTION_COLOR_H

#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>

#include <scenario_intersection/utility.h>

namespace scenario_intersection
{

enum class Color : int
{
  Blank = 0,
  Green,
  Red,
  Yellow,
};

template <>
Color convert<Color>(const std::string&);

std::ostream& operator<<(std::ostream& os, const Color color);

} // namespace scenario_intersection

#endif // INCLUDED_SCENARIO_INTERSECTION_COLOR_H

