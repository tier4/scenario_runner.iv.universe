#include <scenario_actions/action_manager.h>

namespace scenario_actions
{

ActionManager::ActionManager(
  const YAML::Node& actions_node,
  const std::vector<std::string>& actors,
  const std::shared_ptr<ScenarioAPI>& api_ptr)
try
  : actions_node_ {actions_node}
  , actors_ {actors}
  , api_ptr_ {api_ptr}
{
  for (const auto& action_node : actions_node_)
  {
    loadPlugin(action_node);
  }
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to initialize actions.");
}

bool ActionManager::loadPlugin(const YAML::Node& node)
try
{
  const auto type { read_essential<std::string>(node, "Type") + "Action" };

  pluginlib::ClassLoader<scenario_actions::EntityActionBase> loader("scenario_actions", "scenario_actions::EntityActionBase");

  const std::vector<std::string> classes = loader.getDeclaredClasses();

  auto iter =
    std::find_if(classes.begin(), classes.end(),
    [&](std::string c)
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
    plugin->configure(node, actors_, api_ptr_);
    actions_.push_back(plugin);
  }
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to load action plugin.");
}

void ActionManager::run(
  const std::shared_ptr<scenario_intersection::IntersectionManager>& intersection_manager)
try
{
  for (const auto& each : actions_)
  {
    each->run(intersection_manager);
  }
}
catch (...)
{
  SCENARIO_ERROR_RETHROW(CATEGORY(), "Failed to execute action.");
}

} // namespace scenario_actions

