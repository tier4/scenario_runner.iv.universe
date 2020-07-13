#include <scenario_entities/entity_manager.h>
#include <scenario_utility/parse.h>

namespace scenario_entities
{

EntityManager::EntityManager(
  const YAML::Node& node,
  const std::shared_ptr<ScenarioAPI>& api_ptr)
try
  : api_ptr_(api_ptr)
{
  for (auto entity_node : node)
  {
    loadPlugin(entity_node, api_ptr_);
  }
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize entities.");
}

bool EntityManager::setStory(const YAML::Node & story)
try
{
  return std::all_of(
           entities_.begin(), entities_.end(),
           [&](const auto& each)
           {
             return each->setStory(story);
           });
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to set story to entities.");
}

simulation_is EntityManager::update(
    const std::shared_ptr<scenario_intersection::IntersectionManager> & intersection_manager)
try
{
  return
    std::accumulate(
      entities_.begin(), entities_.end(),
      simulation_is::succeeded,
      [&](const auto& lhs, const auto& rhs)
      {
        return lhs and (*rhs).update(intersection_manager);
      });
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to update entities.");
}

bool EntityManager::initialize()
try
{
  return
    std::all_of(
      entities_.begin(), entities_.end(),
      [&](const auto& entity)
      {
        return entity->init();
      });
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize entities.");
}

bool EntityManager::loadPlugin(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr)
try
{
  const auto type { read_essential<std::string>(node, "Type") + "Entity" };

  pluginlib::ClassLoader<scenario_entities::EntityBase> loader("scenario_entities", "scenario_entities::EntityBase");
  std::vector<std::string> classes = loader.getDeclaredClasses();

  const auto iter =
    std::find_if(classes.begin(), classes.end(),
    [&](const std::string& c)
    {
      return loader.getName(c) == type;
    });

  if (iter == classes.end())
  {
    SCENARIO_ERROR_THROW(CATEGORY(), "There is no plugin of type '" << type << "'.");
  }
  else
  {
    auto plugin = loader.createInstance(*iter);
    plugin->configure(node, api_ptr);
    entities_.push_back(plugin);
  }
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to load entity plugin.");
}

}  // namespace scenario_entities
