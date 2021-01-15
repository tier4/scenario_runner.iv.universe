#include <scenario_entities/entity_base.h>
#include <scenario_logger/simple_logger.hpp>

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
  LOG_SIMPLE(info() << "Parse 'Story.Init.Entity[" << name_ << "].InitialStates");
  call_with_essential(init_entity_, "InitialStates", [&](const auto& node)
  {
    const auto type { type_ != "Vehicle" ? boost::to_lower_copy(type_) : "car" };

    api_->addNPC(
      type,
      name_,
      read_essential<geometry_msgs::Pose>(node, "Pose"),
      read_optional<float>(node, "Speed", 0),
      false, // stop_by_vehicle
      read_optional<std::string>(node, "Shift", "Center"));
  });

  LOG_SIMPLE(info() << "Parse 'Story.Init.Entity[" << name_ << "].Actions");
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

