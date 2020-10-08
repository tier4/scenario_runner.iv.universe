#ifndef INCLUDED_SCENARIO_INTERSECTION_ARROW_H
#define INCLUDED_SCENARIO_INTERSECTION_ARROW_H

#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>

#include <scenario_intersection/utility.h>

namespace scenario_intersection
{

enum class Arrow : int
{
  Blank = 0,
  Left,
  LeftRight,
  Right,
  Straight,
  StraightLeft,
  StraightRight,
};

template <>
Arrow convert<Arrow>(const std::string&);

std::ostream& operator<<(std::ostream& os, const Arrow arrow);

} // namespace scenario_intersection

#endif // INCLUDED_SCENARIO_INTERSECTION_ARROW_H

