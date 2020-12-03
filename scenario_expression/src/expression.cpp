// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "scenario_expression/expression.hpp"

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
