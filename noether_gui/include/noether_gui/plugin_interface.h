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
 * @ingroup gui_interfaces
 * @brief Base class for a plugin that can generate a Qt widget
 */
template <typename T>
class WidgetPlugin
{
public:
  using WidgetT = T;
  using Ptr = std::shared_ptr<WidgetPlugin>;
  virtual ~WidgetPlugin() = default;

  virtual QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const = 0;

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
#define EXPORT_TPP_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, tpp)
#define EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, dg)
#define EXPORT_ORIGIN_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, og)
#define EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, mod)
#define EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, mesh)
