#include <scenario_sequence/event.h>

namespace scenario_sequence
{

Event::Event(
  const YAML::Node& event_definition,
  const std::shared_ptr<ScenarioAPI>& simulator,
  const std::shared_ptr<scenario_entities::EntityManager>& entity_manager)
  : event_definition_ {event_definition}
  , simulator_ {simulator}
  , entity_manager_ {entity_manager}
  , name_ {event_definition_["Name"].as<std::string>()}
  , ignited_ {false}
{
  ROS_INFO_STREAM("\e[1;32m          - Name: " << name_ << "\e[0m");

  ROS_INFO_STREAM("\e[1;32m            Actors:\e[0m");
  for (const auto& each : event_definition_["Actors"])
  {
    ROS_INFO_STREAM("\e[1;32m              - " << each << "\e[0m");
    actors_.push_back(each.as<std::string>());
  }

  ROS_INFO_STREAM("\e[1;32m            Actions:\e[0m");
  action_manager_
    = std::make_shared<scenario_actions::ActionManager>(
        event_definition_["Actions"],
        actors_,
        simulator_);

  ROS_INFO_STREAM("\e[1;32m            Condition:\e[0m");
  {
    if (const auto conjunctional_definitions_ {event_definition_["Condition"]["All"]})
    {
      for (const auto& each : conjunctional_definitions_)
      {
        conjunctional_conditions_.push_back(load(each));
      }
    }
    else if (const auto disjunctional_definitions_ {event_definition_["Condition"]["Any"]})
    {
      for (const auto& each : disjunctional_definitions_)
      {
        disjunctional_conditions_.push_back(load(each));
      }
    }
    else // NOTE: If Condition unspecified, the sequence starts unconditionally.
    {
      ignited_ = true;
    }
  }

  // (*action_manager_).run();
}

Event::condition_pointer Event::load(const YAML::Node& node) const
{
  static pluginlib::ClassLoader<scenario_conditions::ConditionBase> loader {
    "scenario_conditions", "scenario_conditions::ConditionBase"
  };

  const auto declared_classes {loader.getDeclaredClasses()};

  const auto type {node["Type"].as<std::string>() + "Condition"};

  const auto iter {
    std::find_if(
      declared_classes.begin(), declared_classes.end(),
      [&](const auto& s)
      {
        return loader.getName(s) == type;
      })
  };

  if (iter != declared_classes.end())
  {
    auto instance {loader.createInstance(*iter)};
    (*instance).configure(node, simulator_);
    return instance;
  }

  return {nullptr};
}

simulation_is Event::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>& intersection_manager)
{
  ROS_INFO_STREAM("\e[1;32m          - Name: " << name_ << "\e[0m");
  ROS_INFO_STREAM("\e[1;32m            Condition:\e[0m");

  if (not ignited_ and not conjunctional_conditions_.empty())
  {
    ROS_INFO_STREAM("\e[1;32m              All:\e[0m");

    ignited_
      = std::accumulate(
          conjunctional_conditions_.begin(), conjunctional_conditions_.end(),
          true,
          [&](const auto& lhs, const auto& rhs)
          {
            // NOTE: return lhs and (rhs ? (*rhs).update(intersection_manager) : false);

            const auto result {(*rhs).update(intersection_manager)};

            ROS_INFO_STREAM("\e[1;32m                - Type: " << (rhs ? (*rhs).getType() : "Error"));
            ROS_INFO_STREAM("\e[1;32m                  Name: " << (rhs ? (*rhs).getName() : "Error"));
            ROS_INFO_STREAM("\e[1;32m                  Currently: " << std::boolalpha << result << "\e[0m");

            return lhs and result;
          });
  }
  else if (not ignited_ and not disjunctional_conditions_.empty())
  {
    ROS_INFO_STREAM("\e[1;32m              Any:\e[0m");

    ignited_
      = std::accumulate(
          disjunctional_conditions_.begin(), disjunctional_conditions_.end(),
          false,
          [&](const auto& lhs, const auto& rhs)
          {
            // NOTE: return lhs or (rhs ? (*rhs).update(intersection_manager) : false);

            const auto result {(*rhs).update(intersection_manager)};

            ROS_INFO_STREAM("\e[1;32m                - Type: " << (rhs ? (*rhs).getType() : "Error"));
            ROS_INFO_STREAM("\e[1;32m                  Name: " << (rhs ? (*rhs).getName() : "Error"));
            ROS_INFO_STREAM("\e[1;32m                  Currently: " << std::boolalpha << result << "\e[0m");

            return lhs or result;
          });
  }

  if (ignited_)
  {
    ROS_INFO_STREAM("\e[1;32m          Ignited\e[0m");
    (*action_manager_).run(intersection_manager);
    return simulation_is::succeeded;
  }
  else
  {
    return simulation_is::ongoing;
  }
}

} // namespace scenario_sequence

