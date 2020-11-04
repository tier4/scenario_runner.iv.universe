#include <condition_plugins/reach_position_condition.h>

namespace condition_plugins
{

ReachPositionCondition::ReachPositionCondition()
  : scenario_conditions::ConditionBase { "ReachPosition" }
{}

bool ReachPositionCondition::configure(
  YAML::Node node,
  std::shared_ptr<ScenarioAPI> api_ptr)
try
{
  node_ = node;

  api_ptr_ = api_ptr;

  name_ = read_optional<std::string>(node_, "Name", name_);

  trigger_ = read_essential<std::string>(node_, "Trigger");

  const auto pose_stamped { read_essential<geometry_msgs::PoseStamped>(node_, "Pose") };

  if (pose_stamped.header.frame_id == "/map")
  {
    target_pose_ = pose_stamped.pose;
  }
  else
  {
    target_pose_ =
      api_ptr_->getRelativePose(
        pose_stamped.header.frame_id, pose_stamped.pose);
  }

  tolerance_ = read_essential<float>(node_, "Tolerance");

  shift_ = read_optional<std::string>(node_, "Shift", "Center");

  keep_ = read_optional<bool>(node_, "Keep", false);

  return configured_ = true;
}
catch (...)
{
  configured_ = false;
  SCENARIO_RETHROW_ERROR_FROM_CONDITION_CONFIGURATION();
}

bool ReachPositionCondition::update(const std::shared_ptr<scenario_intersection::IntersectionManager> &)
{
  if (keep_ and result_)
  {
    return result_;
  }
  else
  {
    if (not configured_)
    {
      SCENARIO_THROW_ERROR_ABOUT_INCOMPLETE_CONFIGURATION();
    }

    description_ =
      std::to_string(
        api_ptr_->getDistanceToArea(
          trigger_, target_pose_, shift_));

    if (api_ptr_->isObjectInArea(
          trigger_, target_pose_, tolerance_, boost::math::constants::two_pi<double>(), shift_))
    {
      SCENARIO_LOG_ABOUT_TOGGLE_CONDITION_RESULT();
      return result_ = true;
    }
    else
    {
      return result_ = false;
    }
  }
}

}  // namespace condition_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(condition_plugins::ReachPositionCondition, scenario_conditions::ConditionBase)
