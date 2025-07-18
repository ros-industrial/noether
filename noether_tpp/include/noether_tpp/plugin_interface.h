#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace noether
{
class Factory;

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
   * @param factory Factory for loading nested tool path planning components
   */
  virtual std::unique_ptr<ComponentT> create(const YAML::Node& config,
                                             std::shared_ptr<const Factory> factory) const = 0;

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

/**
 * @brief Template for a default implementation of Plugin
 * @details This plugin makes a new instance of `DerivedT` and configures it with the input YAML config.
 */
template <typename DerivedT, typename BaseT>
struct PluginImpl : public Plugin<BaseT>
{
  std::unique_ptr<BaseT> create(const YAML::Node& config,
                                std::shared_ptr<const Factory> /*factory*/) const override final
  {
    return std::make_unique<DerivedT>(config.as<DerivedT>());
  }
};

/**
 * @brief Factory for creating tool path planning components from plugins
 * @details This class wraps `boost_plugin_loader::PluginLoader` and stores the loaded plugins internally in order to
 * keep them and the objects they create in scope for the lifetime of this class.
 */
class Factory
{
public:
  Factory();
  Factory(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader);

  MeshModifier::Ptr createMeshModifier(const YAML::Node& config) const;
  ToolPathPlanner::Ptr createToolPathPlanner(const YAML::Node& config) const;
  DirectionGenerator::Ptr createDirectionGenerator(const YAML::Node& config) const;
  OriginGenerator::Ptr createOriginGenerator(const YAML::Node& config) const;
  ToolPathModifier::Ptr createToolPathModifier(const YAML::Node& config) const;

  template <typename PluginT>
  std::vector<std::string> getAvailablePlugins() const
  {
    return loader_->getAvailablePlugins<PluginT>();
  }

protected:
  std::shared_ptr<const boost_plugin_loader::PluginLoader> loader_;

private:
  template <typename PluginT>
  typename PluginT::ComponentT::Ptr create(std::map<std::string, typename PluginT::Ptr>& plugin_map,
                                           const YAML::Node& config) const;

  mutable std::map<std::string, MeshModifierPlugin::Ptr> mesh_modifier_plugins_;
  mutable std::map<std::string, ToolPathPlannerPlugin::Ptr> tool_path_planner_plugins_;
  mutable std::map<std::string, DirectionGeneratorPlugin::Ptr> direction_generator_plugins_;
  mutable std::map<std::string, OriginGeneratorPlugin::Ptr> origin_generator_plugins_;
  mutable std::map<std::string, ToolPathModifierPlugin::Ptr> tool_path_modifier_plugins_;
};

}  // namespace noether

#include <boost_plugin_loader/macros.h>

/**
 * @brief Macro for exporting instances of @ref noether::MeshModifierPlugin "MeshModifierPlugin"
 */
#define EXPORT_MESH_MODIFIER_PLUGIN(DERIVED_CLASS, ALIAS)                                                              \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_MESH_MODIFIER_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::ToolPathPlannerPlugin "ToolPathPlannerPlugin"
 */
#define EXPORT_TOOL_PATH_PLANNER_PLUGIN(DERIVED_CLASS, ALIAS)                                                          \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_TPP_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::DirectionGeneratorPlugin "DirectionGeneratorPlugin"
 */
#define EXPORT_DIRECTION_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS)                                                        \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_DIRECTION_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::OriginGeneratorPlugin "OriginGeneratorPlugin"
 */
#define EXPORT_ORIGIN_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS)                                                           \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_ORIGIN_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::ToolPathModifierPlugin "ToolPathModifierPlugin"
 */
#define EXPORT_TOOL_PATH_MODIFIER_PLUGIN(DERIVED_CLASS, ALIAS)                                                         \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_TOOL_PATH_MODIFIER_SECTION)

/**
 * @brief Macro for defining and exporting a default mesh modifier widget plugin using the PluginImpl class
 */
#define EXPORT_DEFAULT_MESH_MODIFIER_PLUGIN(ComponentT, Alias)                                                         \
  using Plugin_##ComponentT = PluginImpl<ComponentT, MeshModifier>;                                                    \
  EXPORT_MESH_MODIFIER_PLUGIN(Plugin_##ComponentT, Alias)

/**
 * @brief Macro for defining and exporting a default tool path planner widget plugin using the PluginImpl class
 */
#define EXPORT_DEFAULT_TOOL_PATH_PLANNER_PLUGIN(ComponentT, Alias)                                                     \
  using Plugin_##ComponentT = PluginImpl<ComponentT, ToolPathPlanner>;                                                 \
  EXPORT_TOOL_PATH_PLANNER_PLUGIN(Plugin_##ComponentT, Alias)

/**
 * @brief Macro for defining and exporting a default direction generator widget plugin using the PluginImpl class
 */
#define EXPORT_DEFAULT_DIRECTION_GENERATOR_PLUGIN(ComponentT, Alias)                                                   \
  using Plugin_##ComponentT = PluginImpl<ComponentT, DirectionGenerator>;                                              \
  EXPORT_DIRECTION_GENERATOR_PLUGIN(Plugin_##ComponentT, Alias)

/**
 * @brief Macro for defining and exporting a default origin generator widget plugin using the PluginImpl class
 */
#define EXPORT_DEFAULT_ORIGIN_GENERATOR_PLUGIN(ComponentT, Alias)                                                      \
  using Plugin_##ComponentT = PluginImpl<ComponentT, OriginGenerator>;                                                 \
  EXPORT_ORIGIN_GENERATOR_PLUGIN(Plugin_##ComponentT, Alias)

/**
 * @brief Macro for defining and exporting a default tool path modifier widget plugin using the PluginImpl class
 */
#define EXPORT_DEFAULT_TOOL_PATH_MODIFIER_PLUGIN(ComponentT, Alias)                                                    \
  using Plugin_##ComponentT = PluginImpl<ComponentT, ToolPathModifier>;                                                \
  EXPORT_TOOL_PATH_MODIFIER_PLUGIN(Plugin_##ComponentT, Alias)
