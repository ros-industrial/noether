#pragma once

#include <noether_gui/plugin_interface.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <QWidget>

namespace Ui
{
class TPPPipeline;
}

namespace noether
{
template <typename T>
class PluginLoaderWidget;

/**
 * @brief Widget for creating a tool path planning pipeline
 */
class TPPPipelineWidget : public QWidget
{
  Q_OBJECT
public:
  TPPPipelineWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent = nullptr);

  ToolPathPlannerPipeline createPipeline() const;

  void configure(const YAML::Node& config);

private:
  const boost_plugin_loader::PluginLoader loader_;
  PluginLoaderWidget<MeshModifierWidgetPlugin>* mesh_modifier_loader_widget_;
  PluginLoaderWidget<ToolPathModifierWidgetPlugin>* tool_path_modifier_loader_widget_;
  Ui::TPPPipeline* ui_;
};

}  // namespace noether
