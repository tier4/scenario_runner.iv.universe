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
 * NOTE: There is no Conditional.
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

template <template <typename T> typename Comparator>
class Logical;

using And = Logical<std::logical_and>;
using Or  = Logical<std::logical_or>;
using Not = Logical<std::logical_not>;

class Expression
{
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

Expression make_expression(const YAML::Node&);

template <template <typename T> typename Comparator>
class Logical
  : public Expression
{
  friend class Expression;

protected:
  Comparator<bool> compare;

  std::vector<Expression> operands;

  Logical(const Logical& operation)
    : Expression { std::integral_constant<decltype(0), 0>() }
    , compare { operation.compare }
    , operands { operation.operands }
  {}

  Logical(const YAML::Node& operands_node)
    : Expression { std::integral_constant<decltype(0), 0>() }
  {
    std::cout << "\e[1;31m(logical";

    if (not operands_node.IsSequence())
    {
      ROS_ERROR_STREAM("Operands of logical operator must be sequence.");
    }

    for (const auto& each : operands_node)
    {
      std::cout << " ";
      operands.push_back(make_expression(each));
    }

    std::cout << "\e[1;31m)";
  }

  virtual ~Logical() = default;

  // virtual operator bool()
  // {
  //   return evaluate();
  // }

  virtual Expression evaluate()
  {
  }
};

// class LogicalOperator
//   : public Logical
// {
//   friend Expression;
//
//   LogicalOperator(const YAML::Node& node)
//     : Logical { Operator<bool>(), node }
//   {}
//
//   virtual ~LogicalOperator() = default;
// };

// TODO MOVE INTO Expression's CONSTRUCTOR!
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
      std::cout << "\e[1;31m(procedure)";
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
