#include <entity_plugins/bicycle_entity.h>

namespace entity_plugins
{

BicycleEntity::BicycleEntity()
  : scenario_entities::EntityBase {"Bicycle"}
{}

} // namespace entity_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(entity_plugins::BicycleEntity, scenario_entities::EntityBase)

