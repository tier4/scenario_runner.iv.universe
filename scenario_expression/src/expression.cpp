#include <scenario_expression/expression.h>

namespace scenario_expression
{

Expression read(Context& context, const YAML::Node& node)
{
  if (node.IsScalar())
  {
    SCENARIO_ERROR_THROW(CATEGORY(),
      "syntax-error: Scalar cannot appear in this context.\n\n" << node << "\n");
  }
  else if (node.IsSequence())
  {
    SCENARIO_ERROR_THROW(CATEGORY(),
      "syntax-error: Sequence cannot appear in this context.\n\n" << node << "\n");
  }
  else if (node.IsMap()) // is <keyword>
  {
    if (const auto all { node["All"] }) // NOTE: should be 'and'
    {
      return Expression::make<All>(context, all);
    }
    else if (const auto any { node["Any"] }) // NOTE: should be 'or'
    {
      return Expression::make<Any>(context, any);
    }
    else if (const auto type { node["Type"] }) // <procedure call>
    {
      if (const auto params { node["Params"] }) // <action call>
      {
        SCENARIO_ERROR_THROW(CATEGORY(),
          "syntax-error: Procedure-call cannot appear in this context.\n\n" << node << "\n");
      }
      else // <predicate call>
      {
        return Expression::make<Predicate>(context, node);
      }
    }
    else
    {
      SCENARIO_ERROR_THROW(CATEGORY(),
        "syntax-error: Malformed clause.\n\n" << node << "\n");
    }
  }
  else
  {
    SCENARIO_WARN_STREAM(CATEGORY(),
      "syntax-error: An unexpected expression has appeared. It is treated as an empty expression.\n\n" << node << "\n");
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
