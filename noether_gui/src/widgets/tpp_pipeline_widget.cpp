#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include "ui_tpp_pipeline_widget.h"
#include <noether_gui/widgets/plugin_loader_widget.h>
#include <noether_gui/plugin_interface.h>
#include <noether_gui/widgets/collapsible_area_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/mesh_modifiers/compound_modifier.h>
#include <noether_tpp/tool_path_modifiers/compound_modifier.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators.h>
#include <QMenu>
#include <QMessageBox>

template <typename T>
std::vector<typename T::ConstPtr> convert(const QWidgetList& widgets)
{
  std::vector<typename T::ConstPtr> out;
  for (const QWidget* widget : widgets)
  {
    auto derived = dynamic_cast<const noether::BaseWidget<T>*>(widget);
    if (derived)
    {
      out.emplace_back(derived->create());
    }
  }

  return out;
}

namespace noether
{
TPPPipelineWidget::TPPPipelineWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent)
  : QWidget(parent)
  , loader_(std::move(loader))
  , ui_(new Ui::TPPPipeline())
  , mesh_modifier_loader_widget_(new PluginLoaderWidget<MeshModifierWidgetPlugin>(loader_, "Mesh Modifier", this))
  , tool_path_modifier_loader_widget(
        new PluginLoaderWidget<ToolPathModifierWidgetPlugin>(loader_, "Tool Path Modifier", this))
{
  ui_->setupUi(this);

  // Populate the combo boxes
  ui_->combo_box_tpp->addItems(getAvailablePlugins<ToolPathPlannerWidgetPlugin>(loader_));

  // Add the tool path modifier loader widget
  ui_->vertical_layout_tool_path_mod->addWidget(tool_path_modifier_loader_widget);

  // Add the mesh modifier loader widget
  ui_->vertical_layout_mesh_mod->addWidget(mesh_modifier_loader_widget_);

  connect(ui_->combo_box_tpp, &QComboBox::currentTextChanged, [this](const QString& text) {
    if (text.isEmpty())
    {
      overwriteWidget(ui_->group_box_tpp->layout(), ui_->widget_tpp, new QWidget(this));
    }
    else
    {
      try
      {
        auto plugin = loader_.createInstance<ToolPathPlannerWidgetPlugin>(text.toStdString());
        overwriteWidget(ui_->group_box_tpp->layout(), ui_->widget_tpp, plugin->create(this));
      }
      catch (const std::exception& ex)
      {
        QMessageBox::warning(this, "Tool Path Planner Error", QString::fromStdString(ex.what()));
      }
    }
  });
}

ToolPathPlannerPipeline TPPPipelineWidget::createPipeline() const
{
  auto widget_tpp = dynamic_cast<ToolPathPlannerWidget*>(ui_->widget_tpp);
  if (!widget_tpp)
    throw std::runtime_error("No tool path planner specified");

  // Get the mesh modifiers
  MeshModifier::ConstPtr mesh_mod;
  {
    std::vector<MeshModifier::ConstPtr> mesh_mods = convert<MeshModifier>(mesh_modifier_loader_widget_->getWidgets());
    if (mesh_mods.empty())
      mesh_mod = std::make_unique<MeshModifier>();
    else
      mesh_mod = std::make_unique<CompoundMeshModifier>(std::move(mesh_mods));
  }

  // Get the tool path planner
  ToolPathPlanner::ConstPtr tpp = widget_tpp->create();

  // Get the tool path modifiers
  ToolPathModifier::ConstPtr tool_path_mod;
  {
    std::vector<ToolPathModifier::ConstPtr> tool_path_mods =
        convert<ToolPathModifier>(tool_path_modifier_loader_widget->getWidgets());

    if (tool_path_mods.empty())
      tool_path_mod = std::make_unique<const ToolPathModifier>();
    else
      tool_path_mod = std::make_unique<const CompoundModifier>(std::move(tool_path_mods));
  }

  return ToolPathPlannerPipeline(std::move(mesh_mod), std::move(tpp), std::move(tool_path_mod));
}

}  // namespace noether
