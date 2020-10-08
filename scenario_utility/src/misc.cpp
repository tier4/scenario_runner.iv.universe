#include <scenario_utility/misc.h>

// namespace scenario_utility
// {
// inline namespace misc
// {

std::ostream& operator<<(std::ostream& os, const simulation_is& is)
{
  switch (is)
  {
  case simulation_is::failed:
    return os << "Failed";

  case simulation_is::ongoing:
    return os << "Ongoing";

  case simulation_is::succeeded:
    return os << "Succeeded";
  }
}

// } // inline namespace misc
// } // namespace scenario_utility

