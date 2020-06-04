#ifndef INCLUDED_SCENARIO_EXPRESSION_EXPRESSION_H
#define INCLUDED_SCENARIO_EXPRESSION_EXPRESSION_H

#include <functional>
#include <iostream>
#include <type_traits>
#include <utility>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

namespace scenario_expression
{

/* -----------------------------------------------------------------------------
 *
 * <Expression> = <Literal>
 *              | <Logical>
 *              | <Procedure Call>
 *              | <Sequential>
 *              | <Parallel>
 *
 * There is no Conditional.
 *
 * <Logical> = <Logical Operator> [ <Test>* ]
 *
 * <Logical Operator> = <And> | <Or> | <Not>
 *
 * <Test> = <Expression>
 *
 * The value of the test is Boolean, which returns whether the return value of
 * the expression is equal to false or not. Note that the return value of the
 * expression is not necessarily Boolean.
 *
 * <Procedure Call> = <Action Call> | <Predicate Call>
 *
 * -------------------------------------------------------------------------- */

class Logical;

template <template <typename T> typename Operator>
class LogicalOperator;

using And = LogicalOperator<std::logical_and>;
using Or  = LogicalOperator<std::logical_or>;
using Not = LogicalOperator<std::logical_not>;

class Expression
{
  friend Logical;

  friend And;
  friend Or;
  friend Not;

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

  virtual Expression evaluate()
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

class Logical
  : public Expression
{
  friend class Expression;

  template <typename T>
  using Comparator = std::function<bool (const T&, const T&)>;

protected:
  Comparator<bool> compare;

  std::vector<Expression> operands;

  Logical(const Logical& operation)
    : Expression { std::integral_constant<decltype(0), 0>() }
    , compare { operation.compare }
    , operands { operation.operands }
  {}

  Logical(const Comparator<bool>& compare, const YAML::Node& operands_node)
    : Expression { std::integral_constant<decltype(0), 0>() }
    , compare { compare }
  {
    for (const auto& each : operands_node)
    {
      ROS_ERROR_STREAM("OPERAND " << each);
    }
  }

  virtual Expression evaluate()
  {
  }

  virtual ~Logical() = default;
};

template <template <typename T> typename Operator>
class LogicalOperator
  : public Logical
{
  friend Expression;

  LogicalOperator(const YAML::Node& node)
    : Logical { Operator<bool>(), node }
  {}

  virtual ~LogicalOperator() = default;
};

Expression make_expression(const YAML::Node& node)
{
  if (node.IsScalar())
  {
    ROS_ERROR_STREAM("IsScalar " << node);
  }
  else if (node.IsSequence())
  {
    ROS_ERROR_STREAM("IsSequence " << node);
  }
  else if (node.IsMap()) // NOTE: is keyword
  {
    if (const auto node_and { node["All"] })
    {
      return Expression::make<And>(node_and);
    }
    if (const auto node_or { node["Any"] })
    {
      return Expression::make<Or>(node_or);
    }
    else
    {
      ROS_ERROR_STREAM("It may be procedure call " << node);
    }
  }

  return {};
}

} // namespace scenario_expression

namespace std
{

template <>
void swap(scenario_expression::Expression& lhs,
          scenario_expression::Expression& rhs)
{
  lhs.swap(rhs);
}

} // namespace std

#endif // INCLUDED_SCENARIO_EXPRESSION_EXPRESSION_H
