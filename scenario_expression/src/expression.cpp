#include <scenario_expression/expression.h>

namespace scenario_expression
{

Expression read(Context& context, const YAML::Node& node)
{
  if (node.IsScalar())
  {
    ROS_ERROR_STREAM("IsScalar " << node);
  }
  else if (node.IsSequence())
  {
    ROS_ERROR_STREAM("IsSequence " << node);
  }
  else if (node.IsMap()) // is <keyword>
  {
    if (const auto all { node["All"] }) // NOTE: should be 'and'
    {
      return Expression::make<And>(context, all);
    }
    else if (const auto any { node["Any"] }) // NOTE: should be 'or'
    {
      return Expression::make<Or>(context, any);
    }
    else if (const auto type { node["Type"] }) // <procedure call>
    {
      if (const auto params { node["Params"] }) // <action call>
      {
        // std::cout << "\e[1;31m(change " << node_type.as<std::string>() << ")";
      }
      else // <predicate call>
      {
        return Expression::make<Predicate>(context, node);
      }
    }
    else
    {
      ROS_ERROR_STREAM("ERROR!");
    }
  }
  else
  {
    return {};
  }
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
