#include <noether_gui/plugin_interface.h>

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <yaml-cpp/yaml.h>

// Macros for converting a non-string target compile definition into a string
#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)

namespace noether
{
template <>
std::string noether::ToolPathPlannerWidgetPlugin::getSection()
{
  return STRINGIFY(NOETHER_GUI_TPP_SECTION);
}

template <>
std::string DirectionGeneratorWidgetPlugin::getSection()
{
  return STRINGIFY(NOETHER_GUI_DIRECTION_GENERATOR_SECTION);
}

template <>
std::string OriginGeneratorWidgetPlugin::getSection()
{
  return STRINGIFY(NOETHER_GUI_ORIGIN_GENERATOR_SECTION);
}

template <>
std::string ToolPathModifierWidgetPlugin::getSection()
{
  return STRINGIFY(NOETHER_GUI_TOOL_PATH_MODIFIER_SECTION);
}

template <>
std::string MeshModifierWidgetPlugin::getSection()
{
  return STRINGIFY(NOETHER_GUI_MESH_MODIFIER_SECTION);
}
}  // namespace noether

namespace boost_plugin_loader
{
INSTANTIATE_PLUGIN_LOADER(noether::ToolPathPlannerWidgetPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::DirectionGeneratorWidgetPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::OriginGeneratorWidgetPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::ToolPathModifierWidgetPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::MeshModifierWidgetPlugin)

}  // namespace boost_plugin_loader
