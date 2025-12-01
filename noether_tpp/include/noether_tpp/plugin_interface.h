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
 * @ingroup plugins
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
 * @ingroup plugins
 */
using ToolPathPlannerPlugin = Plugin<ToolPathPlanner>;

/**
 * @brief Plugin interface for creating a DirectionGenerator
 * @ingroup plugins
 */
using DirectionGeneratorPlugin = Plugin<DirectionGenerator>;

/**
 * @brief Plugin interface for creating an OriginGenerator
 * @ingroup plugins
 */
using OriginGeneratorPlugin = Plugin<OriginGenerator>;

/**
 * @brief Plugin interface for creating a ToolPathModifier
 * @ingroup plugins
 */
using ToolPathModifierPlugin = Plugin<ToolPathModifier>;

/**
 * @brief Plugin interface for creating a MeshModifier
 * @ingroup plugins
 */
using MeshModifierPlugin = Plugin<MeshModifier>;

/**
 * @brief Template for a simple implementation of Plugin
 * @details This plugin makes a new instance of `DerivedT` and configures it with the input YAML config.
 *
 * Use this class to define a plugin for your custom tool path planning component class if your class can be fully
 * configured using only the `YAML::Node` configuration:
 * @code{.cpp}
 * using Plugin_MyClass = SimplePlugin<MyClass>;
 * @endcode
 *
 * @ingroup plugins
 */
template <typename DerivedT, typename BaseT>
struct SimplePlugin : public Plugin<BaseT>
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
 * @ingroup plugins
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

  template <typename PluginT>
  typename PluginT::ComponentT::Ptr create(const YAML::Node& config) const;
};

}  // namespace noether

#include <boost_plugin_loader/macros.h>

/**
 * @brief Macro for exporting instances of @ref noether::MeshModifierPlugin "MeshModifierPlugin"
 * @ingroup plugins
 */
#define EXPORT_MESH_MODIFIER_PLUGIN(DERIVED_CLASS, ALIAS)                                                              \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_MESH_MODIFIER_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::ToolPathPlannerPlugin "ToolPathPlannerPlugin"
 * @ingroup plugins
 */
#define EXPORT_TOOL_PATH_PLANNER_PLUGIN(DERIVED_CLASS, ALIAS)                                                          \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_TPP_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::DirectionGeneratorPlugin "DirectionGeneratorPlugin"
 * @ingroup plugins
 */
#define EXPORT_DIRECTION_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS)                                                        \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_DIRECTION_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::OriginGeneratorPlugin "OriginGeneratorPlugin"
 * @ingroup plugins
 */
#define EXPORT_ORIGIN_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS)                                                           \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_ORIGIN_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of @ref noether::ToolPathModifierPlugin "ToolPathModifierPlugin"
 * @ingroup plugins
 */
#define EXPORT_TOOL_PATH_MODIFIER_PLUGIN(DERIVED_CLASS, ALIAS)                                                         \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_TOOL_PATH_MODIFIER_SECTION)

/**
 * @brief Macro for defining and exporting a simple mesh modifier plugin using the noether::SimplePlugin class
 * @ingroup plugins
 */
#define EXPORT_SIMPLE_MESH_MODIFIER_PLUGIN(ComponentT, Alias)                                                          \
  using Plugin_##ComponentT = SimplePlugin<ComponentT, MeshModifier>;                                                  \
  EXPORT_MESH_MODIFIER_PLUGIN(Plugin_##ComponentT, Alias)

/**
 * @brief Macro for defining and exporting a simple tool path planner plugin using the noether::SimplePlugin class
 * @ingroup plugins
 */
#define EXPORT_SIMPLE_TOOL_PATH_PLANNER_PLUGIN(ComponentT, Alias)                                                      \
  using Plugin_##ComponentT = SimplePlugin<ComponentT, ToolPathPlanner>;                                               \
  EXPORT_TOOL_PATH_PLANNER_PLUGIN(Plugin_##ComponentT, Alias)

/**
 * @brief Macro for defining and exporting a simple direction generator plugin using the noether::SimplePlugin class
 * @ingroup plugins
 */
#define EXPORT_SIMPLE_DIRECTION_GENERATOR_PLUGIN(ComponentT, Alias)                                                    \
  using Plugin_##ComponentT = SimplePlugin<ComponentT, DirectionGenerator>;                                            \
  EXPORT_DIRECTION_GENERATOR_PLUGIN(Plugin_##ComponentT, Alias)

/**
 * @brief Macro for defining and exporting a simple origin generator plugin using the noether::SimplePlugin class
 * @ingroup plugins
 */
#define EXPORT_SIMPLE_ORIGIN_GENERATOR_PLUGIN(ComponentT, Alias)                                                       \
  using Plugin_##ComponentT = SimplePlugin<ComponentT, OriginGenerator>;                                               \
  EXPORT_ORIGIN_GENERATOR_PLUGIN(Plugin_##ComponentT, Alias)

/**
 * @brief Macro for defining and exporting a simple tool path modifier plugin using the noether::SimplePlugin class
 * @ingroup plugins
 */
#define EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(ComponentT, Alias)                                                     \
  using Plugin_##ComponentT = SimplePlugin<ComponentT, ToolPathModifier>;                                              \
  EXPORT_TOOL_PATH_MODIFIER_PLUGIN(Plugin_##ComponentT, Alias)
