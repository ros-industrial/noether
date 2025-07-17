#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace boost_plugin_loader
{
class PluginLoader;

/** @cond */
template <typename T>
struct has_getSection;
/** @endcond */
}  // namespace boost_plugin_loader

namespace noether
{
/**
 * @brief Base class for a plugin that can generate a tool path planning component (e.g., mesh modifier, tool path
 * planner, tool path modifier).
 */
template <typename T>
class Plugin
{
public:
  /**
   * @brief Typedef for the tool path planning component type
   */
  typedef T ComponentT;
  using Ptr = std::shared_ptr<Plugin>;
  virtual ~Plugin() = default;

  /**
   * @brief Returns a pointer to a configured tool path planning component (e.g., mesh modifier, tool path planner, tool
   * path modifier).
   * @param config YAML configuration node used to configure the component
   */
  virtual std::unique_ptr<ComponentT> create(const YAML::Node& config = {}) const = 0;

private:
  friend class boost_plugin_loader::PluginLoader;
  friend struct boost_plugin_loader::has_getSection<Plugin>;
  static std::string getSection();
};

/**
 * @brief Plugin interface for creating a ToolPathPlanner
 */
using ToolPathPlannerPlugin = Plugin<ToolPathPlanner>;

/**
 * @brief Plugin interface for creating a DirectionGenerator
 */
using DirectionGeneratorPlugin = Plugin<DirectionGenerator>;

/**
 * @brief Plugin interface for creating an OriginGenerator
 */
using OriginGeneratorPlugin = Plugin<OriginGenerator>;

/**
 * @brief Plugin interface for creating a ToolPathModifier
 */
using ToolPathModifierPlugin = Plugin<ToolPathModifier>;

/**
 * @brief Plugin interface for creating a MeshModifier
 */
using MeshModifierPlugin = Plugin<MeshModifier>;

}  // namespace noether

#include <boost_plugin_loader/macros.h>
/**
 * @brief Macro for exporting instances of @ref noether::ToolPathPlannerPlugin "ToolPathPlannerPlugin"
 */
#define EXPORT_TOOL_PATH_PLANNER_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_TPP_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::DirectionGeneratorPlugin "DirectionGeneratorPlugin"
 */
#define EXPORT_DIRECTION_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_DIRECTION_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::OriginGeneratorPlugin "OriginGeneratorPlugin"
 */
#define EXPORT_ORIGIN_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_ORIGIN_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::ToolPathModifierPlugin "ToolPathModifierPlugin"
 */
#define EXPORT_TOOL_PATH_MODIFIER_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_TOOL_PATH_MODIFIER_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::MeshModifierPlugin "MeshModifierPlugin"
 */
#define EXPORT_MESH_MODIFIER_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_MESH_MODIFIER_SECTION)
