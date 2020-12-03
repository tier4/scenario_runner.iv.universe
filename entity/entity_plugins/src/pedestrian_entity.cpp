#include <entity_plugins/pedestrian_entity.hpp>

namespace entity_plugins
{

PedestrianEntity::PedestrianEntity()
  : scenario_entities::EntityBase {"Pedestrian"}
{}

} // namespace entity_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(entity_plugins::PedestrianEntity, scenario_entities::EntityBase)

