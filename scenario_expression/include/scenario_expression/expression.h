#ifndef INCLUDED_SCENARIO_EXPRESSION_EXPRESSION_H
#define INCLUDED_SCENARIO_EXPRESSION_EXPRESSION_H

#include <iostream>
#include <type_traits>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace scenario_expression
{

/* -----------------------------------------------------------------------------
 *
 * <Expression> = <Conditional>
 *              | <Predicate>
 *              | <Action>
 *
 * <Conditional> = <Syntax> [ <Test>* ]
 *
 * <Test> = <Expression>
 *
 * The value of the test is Boolean, which returns whether the return value of
 * the expression is equal to false or not. Note that the return value of the
 * expression is not necessarily Boolean.
 *
 * -------------------------------------------------------------------------- */

class Conditional;
class Predicate;

class Expression
{
  friend class Conditional;
  friend class Predicate;

public:
  Expression()
    : data { nullptr }
    , reference_count { 0 }
  {}

  Expression(const Expression& expression)
    : data { expression.data }
    , reference_count { 0 }
  {
    if (data)
    {
      ++(data->reference_count);
    }
  }

  virtual ~Expression()
  {
    if (data && --(data->reference_count) == 0)
    {
      std::cout << "Expression::~Expression deleting data" << std::endl;
      delete data;
    }
  }

  Expression& operator =(const Expression& rhs)
  {
    Expression e { rhs };
    swap(e);
    return *this;
  }

  virtual Expression operator ()()
  {}

  template <typename T, typename... Ts>
  static auto make(Ts&&... xs)
  {
    Expression e {};
    e.remake(new T { std::forward<decltype(xs)>(xs)... });
    return e;
  }

  void swap(Expression& e) noexcept
  {
    std::swap(data, e.data);
  }

protected:
  Expression(std::integral_constant<decltype(0), 0>)
    : data { nullptr }
    , reference_count { 1 }
  {}

private:
  void remake(Expression* e)
  {
    if (data && --(data->reference_count) == 0)
    {
      delete data;
    }

    data = e;
  }

  Expression* data;

  std::size_t reference_count;
};

class Syntax;
class Test;

class Conditional
  : public Expression
{
  friend class Syntax;
  friend class Test;

public:
  Conditional(const YAML::Node& node)
    : Expression { std::integral_constant<decltype(0), 0>() }
  {}

  Conditional(const Conditional& c)
    : Expression { std::integral_constant<decltype(0), 0>() }
  {}

  virtual ~Conditional()
  {
    std::cout << "Conditional::~Conditional" << std::endl;
  }

  virtual Expression operator ()()
  {}
};

} // namespace scenario_expression

#endif // INCLUDED_SCENARIO_EXPRESSION_EXPRESSION_H
