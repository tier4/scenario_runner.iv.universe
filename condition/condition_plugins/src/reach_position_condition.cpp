#include <condition_plugins/reach_position_condition.h>

namespace condition_plugins
{

ReachPositionCondition::ReachPositionCondition()
  : scenario_conditions::ConditionBase {"ReachPosition"}
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

  target_pose_ = read_essential<geometry_msgs::PoseStamped>(node_, "Pose");

  tolerance_ = read_essential<float>(node_, "Tolerance");

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

    if (target_pose_->header.frame_id == "/map")
    {
      if ((*api_ptr_).isObjectInArea(trigger_, target_pose_->pose, tolerance_, boost::math::constants::two_pi<double>()))
      {
        SCENARIO_LOG_ABOUT_TOGGLE_CONDITION_RESULT();
        return result_ = true;
      }
      else
      {
        return result_ = false;
      }
    }
    else
    {
      auto p = api_ptr_->getRelativePose(target_pose_->header.frame_id,
                                         target_pose_->pose.position.x,
                                         target_pose_->pose.position.y,
                                         target_pose_->pose.position.z,
                                         target_pose_->pose.orientation.x,
                                         target_pose_->pose.orientation.y,
                                         target_pose_->pose.orientation.z,
                                         target_pose_->pose.orientation.w);

      if ((*api_ptr_).isObjectInArea(trigger_, p, tolerance_, boost::math::constants::two_pi<double>()))
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
}

}  // namespace condition_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(condition_plugins::ReachPositionCondition, scenario_conditions::ConditionBase)
