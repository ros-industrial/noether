#pragma once

#include <noether_gui/plugin_interface.h>
#include <noether_gui/widgets/plugin_loader_widget.h>

#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <QWidget>

namespace Ui
{
class TPPPipeline;
}

namespace noether
{
/**
 * @brief Widget for creating a tool path planning pipeline
 */
class TPPPipelineWidget : public QWidget
{
public:
  TPPPipelineWidget(std::shared_ptr<const GuiFactory> factory, QWidget* parent = nullptr);

  ToolPathPlannerPipeline createPipeline() const;

  void configure(const YAML::Node& config);
  void save(YAML::Node& config) const;

protected:
  std::shared_ptr<const GuiFactory> factory_;
  PluginLoaderWidget<MeshModifierWidgetPlugin>* mesh_modifier_loader_widget_;
  PluginLoaderWidget<ToolPathModifierWidgetPlugin>* tool_path_modifier_loader_widget_;
  Ui::TPPPipeline* ui_;
};

}  // namespace noether
