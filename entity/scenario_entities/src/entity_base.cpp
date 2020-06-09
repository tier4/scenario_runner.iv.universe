#include <scenario_entities/entity_base.h>

namespace scenario_entities
{

bool EntityBase::configure(
  const YAML::Node& entity,
  const std::shared_ptr<ScenarioAPI>& api)
try
{
  name_ =
    read_optional<std::string>(
      entity, "Name", type_ + "Entity-" + boost::lexical_cast<std::string>(this));

  return static_cast<bool>(api_ = api);
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(),
    "Failed to configure entity named '" << name_ << "' of type " << type_ << ".");
}

bool EntityBase::setStory(const YAML::Node& story)
try
{
  if (story["Init"])
  {
    init_ = story["Init"];
  }

  if (story["Act"])
  {
    act_ = story["Act"];
  }

  if (story["EndCondition"])
  {
    end_condition_ = story["EndCondition"];
  }

  call_with_essential(init_, "Entity", [&](const auto& node) mutable
  {
    for (const YAML::Node& each : node)
    {
      if (read_essential<std::string>(each, "Name") == name_)
      {
        return init_entity_ = each;
      }
    }

    SCENARIO_ERROR_THROW(CATEGORY(),
      "There is no initialization for entity named '" << name_ << "' of type " << type_ << ".");
  });

  return init_entity_;
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(),
    "Failed to read story to entity named '" << name_ << "' of type " << type_ << ".");
}

bool EntityBase::init()
try
{
  ROS_INFO_STREAM("\e[1;32m      - Name: " << name_ << "\e[0m");

  if (const auto pose {init_entity_["InitialStates"]["Pose"]})
  {
    ROS_INFO_STREAM("\e[1;32m        InitialStates:\e[0m");
    ROS_INFO_STREAM("\e[1;32m          Pose:\e[0m");

    if (const auto pose_stamped {scenario_utility::parse::getPoseStamped(pose)})
    {
      pose_stamped_ = pose_stamped.get();
      pose_stamped_.header.stamp = ros::Time::now();

      ROS_INFO_STREAM("\e[1;32m            Position:");
      ROS_INFO_STREAM("\e[1;32m              X: " << pose_stamped_.pose.position.x);
      ROS_INFO_STREAM("\e[1;32m              Y: " << pose_stamped_.pose.position.y);
      ROS_INFO_STREAM("\e[1;32m              Z: " << pose_stamped_.pose.position.z);
      ROS_INFO_STREAM("\e[1;32m            Orientation:");
      ROS_INFO_STREAM("\e[1;32m              X: " << pose_stamped_.pose.orientation.x);
      ROS_INFO_STREAM("\e[1;32m              Y: " << pose_stamped_.pose.orientation.y);
      ROS_INFO_STREAM("\e[1;32m              Z: " << pose_stamped_.pose.orientation.z);
      ROS_INFO_STREAM("\e[1;32m              W: " << pose_stamped_.pose.orientation.w);
      ROS_INFO_STREAM("\e[1;32m            FrameID: " << pose_stamped_.header.frame_id);
    }
    else
    {
      std::stringstream ss {};
      ss << "Failed to initialize "
         << type_
         << "Entity's InitialStates.Pose. Invalid pose specified.";

      scenario_logger::log.addLog(
        scenario_logger_msgs::Level::LEVEL_ERROR,
        {"simulator"},
        ss.str(),
        name_);
      scenario_logger::log.write();

      throw std::runtime_error {ss.str()};
    }
  }
  else
  {
    std::stringstream ss {};
    ss << "Failed to initialize "
       << type_
       << "Entity's InitialStates.Pose. No pose specified.";

    scenario_logger::log.addLog(
      scenario_logger_msgs::Level::LEVEL_ERROR,
      {"simulator"},
      ss.str(),
      name_);
    scenario_logger::log.write();

    throw std::runtime_error {ss.str()};
  }

  if (const auto speed {init_entity_["InitialStates"]["Speed"]})
  {
    speed_ = speed.as<float>();
    ROS_INFO_STREAM("\e[1;32m          Speed: " << speed << "\e[0m");
  }
  else
  {
    speed_ = 0;
    ROS_INFO_STREAM("\e[0;32m          Speed: " << speed << " (Unspecified)\e[0m");
  }

  if (type_ != "Vehicle") // XXX DIRTY HACK!!!
  {
    (*api_).addNPC(boost::to_lower_copy(type_), name_, pose_stamped_.pose, speed_, false);
  }
  else
  {
    const auto type { type_ != "Vehicle" ? boost::to_lower_copy(type_) : "car" };

    api_->addNPC(
      type,
      name_,
      read_essential<geometry_msgs::Pose>(node, "Pose"),
      read_optional<float>(node, "Speed", 0),
      false,
      read_optional<std::string>(node, "Shift", "Center"));
  });

  call_with_optional(init_entity_, "Actions", [&](const auto& node) mutable
  {
    action_manager_ =
      std::make_shared<scenario_actions::ActionManager>(
        node, std::vector<std::string> { name_ }, api_);

    action_manager_->run(nullptr);
  });

  return true;
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(),
    "Failed to initialize entity named '" << name_ << "' of type " << type_ << ".");
}

[[deprecated]]
simulation_is EntityBase::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager> & intersection_manager)
{
  return simulation_is::succeeded;
}

} // namespace scenario_entities

