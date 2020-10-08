#ifndef SCENARIO_UTILS_MISC_H_INCLUDED
#define SCENARIO_UTILS_MISC_H_INCLUDED

#include <ostream>

// namespace scenario_utility
// {
// inline namespace misc
// {

enum class simulation_is : int
{
  // XXX DON'T SORT THIS!!!
  failed = -1,
  ongoing,
  succeeded,
};

std::ostream& operator<<(std::ostream&, const simulation_is&);

constexpr simulation_is operator &&(const simulation_is& lhs,
                                    const simulation_is& rhs)
{
  using comparable = std::underlying_type<simulation_is>::type;
  return
    static_cast<simulation_is>(
      std::min<comparable>(
        static_cast<comparable>(lhs),
        static_cast<comparable>(rhs)));
}

constexpr simulation_is operator ||(const simulation_is& lhs,
                                    const simulation_is& rhs)
{
  using comparable = std::underlying_type<simulation_is>::type;
  return
    static_cast<simulation_is>(
      std::max<comparable>(
        static_cast<comparable>(lhs),
        static_cast<comparable>(rhs)));
}


static_assert( ( simulation_is::succeeded && simulation_is::succeeded ) == simulation_is::succeeded );
static_assert( ( simulation_is::succeeded && simulation_is::ongoing   ) == simulation_is::ongoing   );
static_assert( ( simulation_is::succeeded && simulation_is::failed    ) == simulation_is::failed    );

static_assert( ( simulation_is::ongoing   && simulation_is::succeeded ) == simulation_is::ongoing   );
static_assert( ( simulation_is::ongoing   && simulation_is::ongoing   ) == simulation_is::ongoing   );
static_assert( ( simulation_is::ongoing   && simulation_is::failed    ) == simulation_is::failed    );

static_assert( ( simulation_is::failed    && simulation_is::succeeded ) == simulation_is::failed    );
static_assert( ( simulation_is::failed    && simulation_is::ongoing   ) == simulation_is::failed    );
static_assert( ( simulation_is::failed    && simulation_is::failed    ) == simulation_is::failed    );


static_assert( ( simulation_is::succeeded || simulation_is::succeeded ) == simulation_is::succeeded );
static_assert( ( simulation_is::succeeded || simulation_is::ongoing   ) == simulation_is::succeeded );
static_assert( ( simulation_is::succeeded || simulation_is::failed    ) == simulation_is::succeeded );

static_assert( ( simulation_is::ongoing   || simulation_is::succeeded ) == simulation_is::succeeded );
static_assert( ( simulation_is::ongoing   || simulation_is::ongoing   ) == simulation_is::ongoing   );
static_assert( ( simulation_is::ongoing   || simulation_is::failed    ) == simulation_is::ongoing   );

static_assert( ( simulation_is::failed    || simulation_is::succeeded ) == simulation_is::succeeded );
static_assert( ( simulation_is::failed    || simulation_is::ongoing   ) == simulation_is::ongoing   );
static_assert( ( simulation_is::failed    || simulation_is::failed    ) == simulation_is::failed    );

// }  inline namespace misc
// }  namespace scenario_utility

#endif  // SCENARIO_UTILS_MISC_H_INCLUDED

