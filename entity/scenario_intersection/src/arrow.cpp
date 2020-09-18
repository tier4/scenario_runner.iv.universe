#include <scenario_intersection/arrow.h>

namespace scenario_intersection
{

template <>
Arrow convert<Arrow>(const std::string& name)
{
  static const std::unordered_map<std::string, Arrow> arrows
  {
    std::make_pair("Blank",         Arrow::Blank),
    std::make_pair("Left",          Arrow::Left),
    std::make_pair("LeftRight",     Arrow::LeftRight),
    std::make_pair("Right",         Arrow::Right),
    std::make_pair("Straight",      Arrow::Straight),
    std::make_pair("StraightLeft",  Arrow::StraightLeft),
    std::make_pair("StraightRight", Arrow::StraightRight),
  };

  return arrows.at(name);
}

std::ostream& operator<<(std::ostream& os, const Arrow arrow)
{
  switch (arrow)
  {
  case Arrow::Left:
    return os << "Left";

  case Arrow::LeftRight:
    return os << "LeftRight";

  case Arrow::Right:
    return os << "Right";

  case Arrow::Straight:
    return os << "Straight";

  case Arrow::StraightLeft:
    return os << "StraightLeft";

  case Arrow::StraightRight:
    return os << "StraightRight";

  default:
    return os << "Blank";
  }
}

} // namespace scenario_intersection

