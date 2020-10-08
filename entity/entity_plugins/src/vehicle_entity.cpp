#include <entity_plugins/vehicle_entity.h>

namespace entity_plugins
{

VehicleEntity::VehicleEntity()
  : scenario_entities::EntityBase {"Vehicle"}
{}

}  // namespace scenario_entities

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(entity_plugins::VehicleEntity, scenario_entities::EntityBase)

