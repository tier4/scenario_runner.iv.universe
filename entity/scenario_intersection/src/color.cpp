#include <scenario_intersection/color.h>

namespace scenario_intersection
{

template <>
Color convert<Color>(const std::string& name)
{
  static const std::unordered_map<std::string, Color> colors
  {
    std::make_pair("Blank",  Color::Blank),
    std::make_pair("Green",  Color::Green),
    std::make_pair("Red",    Color::Red),
    std::make_pair("Yellow", Color::Yellow),
  };

  return colors.at(name);
}

std::ostream& operator<<(std::ostream& os, const Color color)
{
  switch (color)
  {
  case Color::Green:
    return os << "Green";

  case Color::Red:
    return os << "Red";

  case Color::Yellow:
    return os << "Yellow";

  default:
    return os << "Blank";
  }
}


} // namespace scenario_intersection

