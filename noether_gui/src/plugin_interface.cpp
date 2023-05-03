#include <noether_gui/plugin_interface.h>

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <boost_plugin_loader/plugin_loader.hpp>

using namespace noether;

namespace noether
{
template <>
std::string noether::ToolPathPlannerWidgetPlugin::getSection()
{
  return "tpp";
}

template <>
std::string DirectionGeneratorWidgetPlugin::getSection()
{
  return "dg";
}

template <>
std::string OriginGeneratorWidgetPlugin::getSection()
{
  return "og";
}

template <>
std::string ToolPathModifierWidgetPlugin::getSection()
{
  return "mod";
}

template <>
std::string MeshModifierWidgetPlugin::getSection()
{
  return "mesh";
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
