#include <entity_plugins/ego_entity.h>
#include <scenario_logger/simple_logger.hpp>

namespace entity_plugins
{

EgoEntity::EgoEntity()
  : scenario_entities::EntityBase {"Ego"}
{}

bool EgoEntity::configure(
  const YAML::Node& entity,
  const std::shared_ptr<ScenarioAPI>& api)
try
{
  EntityBase::configure(entity, api);

  urdf_ = read_optional<std::string>(entity, "Urdf");

  initial_frame_id_ =
    read_optional<std::string>(
      entity, "InitialFrameId", "ego-initial-pose");

  if (not (*api_).setEgoCarName(name_))
  {
    SCENARIO_ERROR_THROW(CATEGORY(), "Failed to set ego-car-name.");
  }
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(),
    "Failed to configure entity named '" << name_ << "' of type " << type_ << ".");
}

bool EgoEntity::init()
try
{
  LOG_SIMPLE(info() << "Parse 'Story.Init.Entity[" << name_ << "].InitialStates.Speed");
  if (const auto speed_node {init_entity_["InitialStates"]["Speed"]})
  {
    if (not api_->setMaxSpeed(speed_node.as<float>()))
    {
      SCENARIO_ERROR_THROW(CATEGORY(), "Failed to set max-speed.");
    }
  }

  LOG_SIMPLE(info() << "Parse 'Story.Init.Entity[" << name_ << "].InitialStates.InitialSpeed");
  if (const auto initial_speed { init_entity_["InitialStates"]["InitialSpeed"] })
  {
    if (not api_->sendStartVelocity(initial_speed.as<float>()))
    {
      SCENARIO_ERROR_THROW(CATEGORY(), "Failed to send start-velicity.");
    }
  }
  else
  {
    if (not api_->sendStartVelocity(0))
    {
      SCENARIO_ERROR_THROW(CATEGORY(), "Failed to send start-velicity.");
    }
  }

  LOG_SIMPLE(info() << "Parsing 'Story.Entity[" << name_ << "].InitialStates");
  call_with_essential(init_entity_, "InitialStates", [&](const auto& node) mutable
  {
    const auto pose { read_essential<geometry_msgs::Pose>(node, "Pose") };

    if (not api_->sendStartPoint(pose, true, read_optional<std::string>(node, "Shift", "Center")))
    {
      SCENARIO_ERROR_THROW(CATEGORY(), "Failed to send start-point.");
    }

    if (not api_->setFrameId(initial_frame_id_, pose))
    {
      SCENARIO_ERROR_THROW(CATEGORY(), "Failed to set frame-id.");
    }
  });

  call_with_optional(init_entity_, "Actions", [&](const auto& node) mutable
  {
    initial_action_manager_ptr_ =
      std::make_shared<scenario_actions::ActionManager>(
        node, std::vector<std::string> { name_ }, api_);

    initial_action_manager_ptr_->run(nullptr);
  });

  return true;
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(),
    "Failed to initialize entity named '" << name_ << "' of type " << type_ << ".");
}

} // namespace entity_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(entity_plugins::EgoEntity, scenario_entities::EntityBase)
