#include <action_plugins/acceleration_action.h>

namespace action_plugins
{

AccelerationAction::AccelerationAction()
  : EntityActionBase {"Acceleration"}
{}

void AccelerationAction::configure(
  const YAML::Node& node,
  const std::vector<std::string> actors,
  const std::shared_ptr<ScenarioAPI>& simulator)
{
  node_ = node;
  actors_ = actors;
  api_ptr_ = api_ptr;

  if (actors_.empty())
  {
    SCENARIO_WARNING_ABOUT_NO_ACTORS_SPECIFIED();
  }

  name_ = read_optional<std::string>(node_, "Name", name_);

  if (const auto min {node_["Params"]["Min"]})
  {
    const auto value { min.as<float>() };

    if (0 < value)
    {
      ROS_WARN_STREAM("'Min' value of Acceleration Action should be negative (You specified positive value " << value << ")");
    }

    min_ = value;
  }
  else
  {
    static_assert(std::numeric_limits<float>::has_signaling_NaN);

    min_ = read_optional<float>(node, "Min", std::numeric_limits<float>::signaling_NaN());
    max_ = read_optional<float>(node, "Max", std::numeric_limits<float>::signaling_NaN());

  if (const auto max {node_["Params"]["Max"]})
  {
    const auto value { max.as<float>() };

    if (value < 0)
    {
      ROS_WARN_STREAM("'Max' value of Acceleration Action should be positive (You specified negative value " << value << ")");
    }

    max_ = value;
  }
  else
  {
    static_assert(std::numeric_limits<float>::has_signaling_NaN);
    max_ = std::numeric_limits<float>::signaling_NaN();
  }

    if (max_ < 0)
    {
      SCENARIO_WARN_STREAM(CATEGORY(),
        "'Max' value of Acceleration Action should be positive (You specified negative value " << max_ << ")");
    }
  });
}
catch (...)
{
  SCENARIO_RETHROW_ERROR_FROM_ACTION_CONFIGURATION();
}

auto AccelerationAction::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
  -> void
{
  for (const auto& actor : actors_)
  {
    if (not std::isnan(min_) and not (*api_ptr_).changeNPCAccelMin(actor, min_))
    {
      if (not std::isnan(min_))
      {
        if (not (*api_ptr_).changeNPCAccelMin(actor, min_))
        {
          std::stringstream ss {};

          ss << type_ << "Action failed to change " << actor << "'s minimum acceleration (Note: This action cannot be used for Ego Type entities).";

          scenario_logger::log.addLog(
            scenario_logger_msgs::Level::LEVEL_ERROR,
            {"simulator"},
            ss.str(),
            name_);

          throw std::runtime_error {ss.str()};
        }
      }

      if (not std::isnan(max_))
      {
        if (not (*api_ptr_).changeNPCAccelMax(actor, max_))
        {
          std::stringstream ss {};

          ss << type_ << "Action failed to change " << actor << "'s maximum acceleration (Note: This action cannot be used for Ego Type entities).";

          scenario_logger::log.addLog(
            scenario_logger_msgs::Level::LEVEL_ERROR,
            {"simulator"},
            ss.str(),
            name_);

    if (not std::isnan(max_) and not (*api_ptr_).changeNPCAccelMax(actor, max_))
    {
      SCENARIO_ERROR_THROW(CATEGORY(),
        type_ << "Action failed to change " << actor << "'s maximum acceleration (Note: This action cannot be used for Ego Type entities).");
    }
  }
}

} // namespace action_plugins

PLUGINLIB_EXPORT_CLASS(action_plugins::AccelerationAction, scenario_actions::EntityActionBase)

