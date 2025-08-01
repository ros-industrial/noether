#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>

#include <boost_plugin_loader/plugin_loader.hpp>

using namespace noether;

namespace noether
{
/** @cond */
template <>
std::string noether::ToolPathPlannerPlugin::getSection()
{
  return NOETHER_TPP_SECTION;
}
/** @endcond */

/** @cond */
template <>
std::string DirectionGeneratorPlugin::getSection()
{
  return NOETHER_DIRECTION_GENERATOR_SECTION;
}
/** @endcond */

/** @cond */
template <>
std::string OriginGeneratorPlugin::getSection()
{
  return NOETHER_ORIGIN_GENERATOR_SECTION;
}
/** @endcond */

/** @cond */
template <>
std::string ToolPathModifierPlugin::getSection()
{
  return NOETHER_TOOL_PATH_MODIFIER_SECTION;
}
/** @endcond */

/** @cond */
template <>
std::string MeshModifierPlugin::getSection()
{
  return NOETHER_MESH_MODIFIER_SECTION;
}
/** @endcond */

Factory::Factory()
{
  auto loader = std::make_shared<boost_plugin_loader::PluginLoader>();
  loader->search_libraries.insert(NOETHER_PLUGIN_LIB);
  loader->search_libraries_env = NOETHER_PLUGIN_LIBS_ENV;
  loader->search_paths_env = NOETHER_PLUGIN_PATHS_ENV;
  loader_ = loader;
}

Factory::Factory(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader) : loader_(loader) {}

template <typename PluginT>
typename PluginT::ComponentT::Ptr Factory::create(std::map<std::string, typename PluginT::Ptr>& plugin_map,
                                                  const YAML::Node& config) const
{
  auto name = YAML::getMember<std::string>(config, "name");
  if (plugin_map.find(name) == plugin_map.end())
    plugin_map[name] = loader_->createInstance<PluginT>(name);

  auto this_shared = std::shared_ptr<const Factory>(this, [](const Factory*) {});
  return plugin_map[name]->create(config, this_shared);
}

MeshModifier::Ptr Factory::createMeshModifier(const YAML::Node& config) const
{
  return create<MeshModifierPlugin>(mesh_modifier_plugins_, config);
}

ToolPathPlanner::Ptr Factory::createToolPathPlanner(const YAML::Node& config) const
{
  return create<ToolPathPlannerPlugin>(tool_path_planner_plugins_, config);
}

OriginGenerator::Ptr Factory::createOriginGenerator(const YAML::Node& config) const
{
  return create<OriginGeneratorPlugin>(origin_generator_plugins_, config);
}

DirectionGenerator::Ptr Factory::createDirectionGenerator(const YAML::Node& config) const
{
  return create<DirectionGeneratorPlugin>(direction_generator_plugins_, config);
}

ToolPathModifier::Ptr Factory::createToolPathModifier(const YAML::Node& config) const
{
  return create<ToolPathModifierPlugin>(tool_path_modifier_plugins_, config);
}

template std::vector<std::string> Factory::getAvailablePlugins<MeshModifierPlugin>() const;
template std::vector<std::string> Factory::getAvailablePlugins<ToolPathPlannerPlugin>() const;
template std::vector<std::string> Factory::getAvailablePlugins<DirectionGeneratorPlugin>() const;
template std::vector<std::string> Factory::getAvailablePlugins<OriginGeneratorPlugin>() const;
template std::vector<std::string> Factory::getAvailablePlugins<ToolPathModifierPlugin>() const;

}  // namespace noether

namespace boost_plugin_loader
{
INSTANTIATE_PLUGIN_LOADER(noether::ToolPathPlannerPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::DirectionGeneratorPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::OriginGeneratorPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::ToolPathModifierPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::MeshModifierPlugin)

}  // namespace boost_plugin_loader
