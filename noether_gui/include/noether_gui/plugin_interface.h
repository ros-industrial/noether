#pragma once

#include <noether_gui/widgets.h>

#include <memory>
#include <noether_tpp/plugin_interface.h>
#include <string>
#include <yaml-cpp/yaml.h>

namespace noether
{
class WidgetFactory;

/**
 * @ingroup gui_interfaces
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
                             std::shared_ptr<const WidgetFactory> factory,
                             QWidget* parent = nullptr) const = 0;
};

/**
 * @ingroup gui_interfaces
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
 * @ingroup gui_interfaces
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
 * @ingroup gui_interfaces
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
 * @ingroup gui_interfaces
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
 * @ingroup gui_interfaces
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
 * @ingroup gui_interfaces
 * @brief Template for a simple implementation of WidgetPlugin
 * @details This plugin makes a new instance of `WidgetT` and attempts to configure it with the input YAML config.
 */
template <typename WidgetT, typename WidgetPluginT>
struct SimpleWidgetPlugin : WidgetPluginT
{
  BaseWidget* create(const YAML::Node& config,
                     std::shared_ptr<const WidgetFactory> /*loader*/,
                     QWidget* parent = nullptr) const override final
  {
    auto widget = new WidgetT(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
    {
      try
      {
        widget->configure(config);
      }
      catch (const std::exception&)
      {
        // Delete the widget to prevent it from showing up in the GUI outside of the appropriate layout
        widget->deleteLater();
        throw;
      }
    }

    return widget;
  }
};

/**
 * @ingroup gui_interfaces
 * @brief Extends the Factory class to be able to load widget plugins for the GUI
 */
class WidgetFactory : public Factory
{
public:
  WidgetFactory();
  WidgetFactory(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader);

  template <typename PluginT>
  BaseWidget* createWidget(const std::string& name, const YAML::Node& config, QWidget* parent = nullptr) const;

  BaseWidget* createMeshModifierWidget(const std::string& name,
                                       const YAML::Node& config = {},
                                       QWidget* parent = nullptr) const;

  BaseWidget* createToolPathPlannerWidget(const std::string& name,
                                          const YAML::Node& config = {},
                                          QWidget* parent = nullptr) const;

  BaseWidget* createDirectionGeneratorWidget(const std::string& name,
                                             const YAML::Node& config = {},
                                             QWidget* parent = nullptr) const;

  BaseWidget* createOriginGeneratorWidget(const std::string& name,
                                          const YAML::Node& config = {},
                                          QWidget* parent = nullptr) const;

  BaseWidget* createToolPathModifierWidget(const std::string& name,
                                           const YAML::Node& config = {},
                                           QWidget* parent = nullptr) const;
};

}  // namespace noether

#include <boost_plugin_loader/macros.h>

/**
 * @brief Macro for exporting instances of mesh modifier plugins
 * @ingroup gui_interfaces
 */
#define EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                       \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_MESH_MODIFIER_SECTION)

/**
 * @brief Macro for exporting instances of tool path planner plugins
 * @ingroup gui_interfaces
 */
#define EXPORT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                   \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_TPP_SECTION)

/**
 * @brief Macro for exporting instances of direction generator plugins
 * @ingroup gui_interfaces
 */
#define EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                 \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_DIRECTION_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of origni generator plugins
 * @ingroup gui_interfaces
 */
#define EXPORT_ORIGIN_GENERATOR_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                    \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_ORIGIN_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of tool path modifier plugins
 * @ingroup gui_interfaces
 */
#define EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                  \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_TOOL_PATH_MODIFIER_SECTION)

/**
 * @brief Macro for defining and exporting a simple mesh modifier widget plugin using the noether::SimpleWidgetPlugin
 * class
 * @ingroup gui_interfaces
 */
#define EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(WidgetT, Alias)                                                      \
  using WidgetPlugin_##WidgetT = SimpleWidgetPlugin<WidgetT, MeshModifierWidgetPlugin>;                                \
  EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(WidgetPlugin_##WidgetT, Alias)

/**
 * @brief Macro for defining and exporting a simple tool path planner widget plugin using the
 * noether::SimpleWidgetPlugin class
 * @ingroup gui_interfaces
 */
#define EXPORT_SIMPLE_TOOL_PATH_PLANNER_WIDGET_PLUGIN(WidgetT, Alias)                                                  \
  using WidgetPlugin_##WidgetT = SimpleWidgetPlugin<WidgetT, ToolPathPlannerWidgetPlugin>;                             \
  EXPORT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(WidgetPlugin_##WidgetT, Alias)

/**
 * @brief Macro for defining and exporting a simple direction generator widget plugin using the
 * noether::SimpleWidgetPlugin class
 * @ingroup gui_interfaces
 */
#define EXPORT_SIMPLE_DIRECTION_GENERATOR_WIDGET_PLUGIN(WidgetT, Alias)                                                \
  using WidgetPlugin_##WidgetT = SimpleWidgetPlugin<WidgetT, DirectionGeneratorWidgetPlugin>;                          \
  EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(WidgetPlugin_##WidgetT, Alias)

/**
 * @brief Macro for defining and exporting a simple origin generator widget plugin using the noether::SimpleWidgetPlugin
 * class
 * @ingroup gui_interfaces
 */
#define EXPORT_SIMPLE_ORIGIN_GENERATOR_WIDGET_PLUGIN(WidgetT, Alias)                                                   \
  using WidgetPlugin_##WidgetT = SimpleWidgetPlugin<WidgetT, OriginGeneratorWidgetPlugin>;                             \
  EXPORT_ORIGIN_GENERATOR_WIDGET_PLUGIN(WidgetPlugin_##WidgetT, Alias)

/**
 * @brief Macro for defining and exporting a simple tool path modifier widget plugin using the
 * noether::SimpleWidgetPlugin class
 * @ingroup gui_interfaces
 */
#define EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(WidgetT, Alias)                                                 \
  using WidgetPlugin_##WidgetT = SimpleWidgetPlugin<WidgetT, ToolPathModifierWidgetPlugin>;                            \
  EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(WidgetPlugin_##WidgetT, Alias)
