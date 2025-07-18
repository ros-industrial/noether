#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/plugin_interface.h>
#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace noether
{
class GuiFactory;

/**
 * @ingroup gui_interfaces_plugins
 * @brief Base class for a plugin that can generate a `BaseWidget<T>` for configuring a tool path planning component
 * (e.g., mesh modifier, tool path planner, tool path modifier).
 */
class WidgetPlugin
{
public:
  using Ptr = std::shared_ptr<WidgetPlugin>;
  virtual ~WidgetPlugin() = default;

  /**
   * @brief Returns a pointer to a configured `BaseWidget<T>`: a widget that can configure a tool path planning
   * component (e.g., mesh modifier, tool path planner, tool path modifier).
   * @param config YAML configuration node used to set the initial values of the widget
   * @param factory Factory for loading nested tool path planning components
   * @param parent Parent widget
   */
  virtual BaseWidget* create(const YAML::Node& config,
                             std::shared_ptr<const GuiFactory> factory,
                             QWidget* parent = nullptr) const = 0;
};

/**
 * @brief WidgetPlugin implementation specifically for BaseWidget instances that configure mesh modifiers
 */
class MeshModifierWidgetPlugin : public WidgetPlugin
{
private:
  friend class boost_plugin_loader::PluginLoader;
  friend struct boost_plugin_loader::has_getSection<MeshModifierWidgetPlugin>;
  static std::string getSection();
};

/**
 * @brief WidgetPlugin implementation specifically for BaseWidget instances that configure tool path planners
 */
class ToolPathPlannerWidgetPlugin : public WidgetPlugin
{
private:
  friend class boost_plugin_loader::PluginLoader;
  friend struct boost_plugin_loader::has_getSection<ToolPathPlannerWidgetPlugin>;
  static std::string getSection();
};

/**
 * @brief WidgetPlugin implementation specifically for BaseWidget instances that configure direction generators
 */
class DirectionGeneratorWidgetPlugin : public WidgetPlugin
{
private:
  friend class boost_plugin_loader::PluginLoader;
  friend struct boost_plugin_loader::has_getSection<DirectionGeneratorWidgetPlugin>;
  static std::string getSection();
};

/**
 * @brief WidgetPlugin implementation specifically for BaseWidget instances that configure origin generators
 */
class OriginGeneratorWidgetPlugin : public WidgetPlugin
{
private:
  friend class boost_plugin_loader::PluginLoader;
  friend struct boost_plugin_loader::has_getSection<OriginGeneratorWidgetPlugin>;
  static std::string getSection();
};

/**
 * @brief WidgetPlugin implementation specifically for BaseWidget instances that configure tool path modifiers
 */
class ToolPathModifierWidgetPlugin : public WidgetPlugin
{
private:
  friend class boost_plugin_loader::PluginLoader;
  friend struct boost_plugin_loader::has_getSection<ToolPathModifierWidgetPlugin>;
  static std::string getSection();
};

/**
 * @brief Template for a default implementation of WidgetPlugin
 * @details This plugin makes a new instance of `WidgetT` and attempts to configure it with the input YAML config.
 */
template <typename WidgetT, typename WidgetPluginT>
struct WidgetPluginImpl : WidgetPluginT
{
  BaseWidget* create(const YAML::Node& config,
                     std::shared_ptr<const GuiFactory> /*loader*/,
                     QWidget* parent = nullptr) const override final
  {
    auto widget = new WidgetT(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

/**
 * @brief Extends the Factory class to be able to load GUI plugins
 */
class GuiFactory : public Factory
{
public:
  GuiFactory();
  GuiFactory(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader);

  template <typename PluginT>
  BaseWidget* createWidget(const std::string& name, const YAML::Node& config, QWidget* parent = nullptr) const;

  BaseWidget* createMeshModifierWidget(const std::string& name, const YAML::Node& config = {}, QWidget* parent = nullptr) const;
  BaseWidget* createToolPathPlannerWidget(const std::string& name, const YAML::Node& config = {}, QWidget* parent = nullptr) const;
  BaseWidget* createDirectionGeneratorWidget(const std::string& name, const YAML::Node& config = {}, QWidget* parent = nullptr) const;
  BaseWidget* createOriginGeneratorWidget(const std::string& name, const YAML::Node& config = {}, QWidget* parent = nullptr) const;
  BaseWidget* createToolPathModifierWidget(const std::string& name, const YAML::Node& config = {}, QWidget* parent = nullptr) const;

protected:
  mutable std::map<std::string, WidgetPlugin::Ptr> widget_plugins_;
};

}  // namespace noether

#include <boost_plugin_loader/macros.h>

/**
 * @brief Macro for exporting instances of mesh modifier plugins
 * @ingroup gui_widgets_mesh_modifiers
 */
#define EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                       \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_MESH_MODIFIER_SECTION)

/**
 * @brief Macro for exporting instances of tool path planner plugins
 * @ingroup gui_widgets_tool_path_planners
 */
#define EXPORT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                   \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_TPP_SECTION)

/**
 * @brief Macro for exporting instances of direction generator plugins
 * @ingroup gui_widgets_tool_path_planners
 */
#define EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                 \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_DIRECTION_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of origni generator plugins
 * @ingroup gui_widgets_tool_path_planners
 */
#define EXPORT_ORIGIN_GENERATOR_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                    \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_ORIGIN_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of tool path modifier plugins
 * @ingroup gui_widgets_tool_path_modifiers
 */
#define EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                  \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_TOOL_PATH_MODIFIER_SECTION)

/**
 * @brief Macro for defining and exporting a default mesh modifier widget plugin using the WidgetPluginImpl class
 */
#define EXPORT_DEFAULT_MESH_MODIFIER_WIDGET_PLUGIN(WidgetT, Alias)                                                     \
  using Plugin_##WidgetT = WidgetPluginImpl<WidgetT, MeshModifierWidgetPlugin>;                                        \
  EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(Plugin_##WidgetT, Alias)

/**
 * @brief Macro for defining and exporting a default tool path planner widget plugin using the WidgetPluginImpl class
 */
#define EXPORT_DEFAULT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(WidgetT, Alias)                                                 \
  using Plugin_##WidgetT = WidgetPluginImpl<WidgetT, ToolPathPlannerWidgetPlugin>;                                     \
  EXPORT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(Plugin_##WidgetT, Alias)

/**
 * @brief Macro for defining and exporting a default direction generator widget plugin using the WidgetPluginImpl class
 */
#define EXPORT_DEFAULT_DIRECTION_GENERATOR_WIDGET_PLUGIN(WidgetT, Alias)                                               \
  using Plugin_##WidgetT = WidgetPluginImpl<WidgetT, DirectionGeneratorWidgetPlugin>;                                  \
  EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(Plugin_##WidgetT, Alias)

/**
 * @brief Macro for defining and exporting a default origin generator widget plugin using the WidgetPluginImpl class
 */
#define EXPORT_DEFAULT_ORIGIN_GENERATOR_WIDGET_PLUGIN(WidgetT, Alias)                                                  \
  using Plugin_##WidgetT = WidgetPluginImpl<WidgetT, OriginGeneratorWidgetPlugin>;                                     \
  EXPORT_ORIGIN_GENERATOR_WIDGET_PLUGIN(Plugin_##WidgetT, Alias)

/**
 * @brief Macro for defining and exporting a default tool path modifier widget plugin using the WidgetPluginImpl class
 */
#define EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(WidgetT, Alias)                                                \
  using Plugin_##WidgetT = WidgetPluginImpl<WidgetT, ToolPathModifierWidgetPlugin>;                                    \
  EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(Plugin_##WidgetT, Alias)
