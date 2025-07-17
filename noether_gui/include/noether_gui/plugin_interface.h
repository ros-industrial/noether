#pragma once

#include <noether_gui/widgets.h>
#include <string>
#include <memory>

class QWidget;

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
 * @ingroup gui_interfaces_plugins
 * @brief Base class for a plugin that can generate a `BaseWidget<T>` for configuring a tool path planning component
 * (e.g., mesh modifier, tool path planner, tool path modifier).
 */
template <typename T>
class WidgetPlugin
{
public:
  /**
   * @brief Typedef for the `BaseWidget<T>` type
   */
  typedef T BaseWidgetT;
  using Ptr = std::shared_ptr<WidgetPlugin>;
  virtual ~WidgetPlugin() = default;

  /**
   * @brief Returns a pointer to a configured `BaseWidget<T>`: a widget that can configure a tool path planning
   * component (e.g., mesh modifier, tool path planner, tool path modifier).
   * @param parent
   * @param config YAML configuration node used to set the initial values of the widget
   */
  virtual BaseWidgetT* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const = 0;

private:
  friend class boost_plugin_loader::PluginLoader;
  friend struct boost_plugin_loader::has_getSection<WidgetPlugin>;
  static std::string getSection();
};

/**
 * @ingroup gui_interfaces_plugins
 * @brief Plugin interface for creating a ToolPathPlannerWidget
 */
using ToolPathPlannerWidgetPlugin = WidgetPlugin<ToolPathPlannerWidget>;

/**
 * @ingroup gui_interfaces_plugins
 * @brief Plugin interface for creating a DirectionGeneratorWidget
 */
using DirectionGeneratorWidgetPlugin = WidgetPlugin<DirectionGeneratorWidget>;

/**
 * @ingroup gui_interfaces_plugins
 * @brief Plugin interface for creating an OriginGeneratorWidget
 */
using OriginGeneratorWidgetPlugin = WidgetPlugin<OriginGeneratorWidget>;

/**
 * @ingroup gui_interfaces_plugins
 * @brief Plugin interface for creating a ToolPathModifierWidget
 */
using ToolPathModifierWidgetPlugin = WidgetPlugin<ToolPathModifierWidget>;

/**
 * @ingroup gui_interfaces_plugins
 * @brief Plugin interface for creating a MeshModifierWidget
 */
using MeshModifierWidgetPlugin = WidgetPlugin<MeshModifierWidget>;

}  // namespace noether

#include <boost_plugin_loader/macros.h>
/**
 * @brief Macro for exporting instances of ToolPathPlannerWidgetPlugin
 * @ingroup gui_widgets_tool_path_planners
 */
#define EXPORT_TPP_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                                 \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_TPP_SECTION)

/**
 * @brief Macro for exporting instances of DirectionGeneratorWidgetPlugin
 * @ingroup gui_widgets_tool_path_planners
 */
#define EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                 \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_DIRECTION_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of OriginGeneratorWidgetPlugin
 * @ingroup gui_widgets_tool_path_planners
 */
#define EXPORT_ORIGIN_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS)                                                    \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_ORIGIN_GENERATOR_SECTION)

/**
 * @brief Macro for exporting instances of ToolPathModifierWidgetPlugin
 * @ingroup gui_widgets_tool_path_modifiers
 */
#define EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                  \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_TOOL_PATH_MODIFIER_SECTION)

/**
 * @brief Macro for exporting instances of MeshModifierWidgetPlugin
 * @ingroup gui_widgets_mesh_modifiers
 */
#define EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS)                                                       \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, NOETHER_GUI_MESH_MODIFIER_SECTION)
