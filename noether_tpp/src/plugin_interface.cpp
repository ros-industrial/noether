#include <noether_tpp/plugin_interface.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <yaml-cpp/yaml.h>

using namespace noether;

namespace noether
{
template <>
std::string noether::ToolPathPlannerPlugin::getSection()
{
  return NOETHER_TPP_SECTION;
}

template <>
std::string DirectionGeneratorPlugin::getSection()
{
  return NOETHER_DIRECTION_GENERATOR_SECTION;
}

template <>
std::string OriginGeneratorPlugin::getSection()
{
  return NOETHER_ORIGIN_GENERATOR_SECTION;
}

template <>
std::string ToolPathModifierPlugin::getSection()
{
  return NOETHER_TOOL_PATH_MODIFIER_SECTION;
}

template <>
std::string MeshModifierPlugin::getSection()
{
  return NOETHER_MESH_MODIFIER_SECTION;
}
}  // namespace noether

namespace boost_plugin_loader
{
INSTANTIATE_PLUGIN_LOADER(noether::ToolPathPlannerPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::DirectionGeneratorPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::OriginGeneratorPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::ToolPathModifierPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::MeshModifierPlugin)

}  // namespace boost_plugin_loader
