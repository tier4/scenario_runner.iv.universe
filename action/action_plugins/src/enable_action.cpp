#include <action_plugins/enable_action.h>

namespace action_plugins
{

EnableAction::EnableAction()
  : EntityActionBase { "Enable" }
{}

void EnableAction::configure(
  const YAML::Node& node,
  const std::vector<std::string> actors,
  const std::shared_ptr<ScenarioAPI>& simulator)
{
  node_ = node;
  actors_ = actors;
  api_ptr_ = simulator;
}

void EnableAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  ROS_INFO_STREAM("\e[1;32m          - Type: " << type_ << "\e[0m");

  if (const auto name { node_["Name"] })
  {
    name_ = name.as<std::string>();
  }
  else
  {
    name_ = type_ + "Action-" + boost::lexical_cast<std::string>(this);
  }

  ROS_INFO_STREAM("\e[1;32m            Name: " << name_ << "\e[0m");

  ROS_INFO_STREAM("\e[1;32m            Params:\e[0m");

  if (const auto feature { node_["Params"]["AutomaticEmergencyBraking"] })
  {
    for (const auto& actor : actors_)
    {
      ROS_INFO_STREAM("\e[1;32m              AutomaticEmergencyBraking: " << std::boolalpha << feature.as<bool>() << "\e[0m");
      (*api_ptr_).changeNPCConsiderVehicle(actor, feature.as<bool>());
    }
  }
}

}

PLUGINLIB_EXPORT_CLASS(action_plugins::EnableAction, scenario_actions::EntityActionBase)
