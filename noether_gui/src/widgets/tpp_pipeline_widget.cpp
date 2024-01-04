#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include "ui_tpp_pipeline_widget.h"
#include <noether_gui/widgets/plugin_loader_widget.h>
#include <noether_gui/plugin_interface.h>
#include <noether_gui/widgets/collapsible_area_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/mesh_modifiers/compound_modifier.h>
#include <noether_tpp/tool_path_modifiers/compound_modifier.h>
#include <QMenu>
#include <QMessageBox>
#include <yaml-cpp/yaml.h>

static const std::string MESH_MODIFIERS_KEY = "mesh_modifiers";
static const std::string TOOL_PATH_PLANNER_KEY = "tool_path_planner";
static const std::string TOOL_PATH_MODIFIERS_KEY = "tool_path_modifiers";

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
  , tool_path_modifier_loader_widget_(
        new PluginLoaderWidget<ToolPathModifierWidgetPlugin>(loader_, "Tool Path Modifier", this))
{
  ui_->setupUi(this);

  // Populate the combo boxes
  ui_->combo_box_tpp->addItems(getAvailablePlugins<ToolPathPlannerWidgetPlugin>(loader_));

  // Add the tool path modifier loader widget
  {
    auto layout = new QVBoxLayout();
    layout->addWidget(tool_path_modifier_loader_widget_);
    ui_->tab_tool_path_modifier->setLayout(layout);
  }

  // Add the mesh modifier loader widget
  {
    auto layout = new QVBoxLayout();
    layout->addWidget(mesh_modifier_loader_widget_);
    ui_->tab_mesh_modifier->setLayout(layout);
  }

  connect(ui_->push_button_select_tpp, &QPushButton::clicked, [this](const bool /*checked*/) {
    try
    {
      const QString text = ui_->combo_box_tpp->currentText();
      ui_->group_box_tpp->setTitle(text);
      if (text.isEmpty())
      {
        ui_->scroll_area->setWidget(new QWidget(this));
      }
      else
      {
        auto plugin = loader_.createInstance<ToolPathPlannerWidgetPlugin>(text.toStdString());
        ui_->scroll_area->setWidget(plugin->create(this));
      }
    }
    catch (const std::exception& ex)
    {
      std::stringstream ss;
      printException(ex, ss);
      QMessageBox::warning(this, "Configuration Error", QString::fromStdString(ss.str()));
    }
  });
}

void TPPPipelineWidget::configure(const YAML::Node& config)
{
  try
  {
    // Mesh modifier
    mesh_modifier_loader_widget_->configure(config[MESH_MODIFIERS_KEY]);

    // Tool path planner
    try
    {
      auto tpp_config = config[TOOL_PATH_PLANNER_KEY];
      auto name = getEntry<std::string>(tpp_config, "name");
      auto plugin = loader_.createInstance<ToolPathPlannerWidgetPlugin>(name);
      ui_->scroll_area->setWidget(plugin->create(this, tpp_config));
      ui_->group_box_tpp->setTitle(QString::fromStdString(name));
    }
    catch (const std::exception&)
    {
      std::throw_with_nested(std::runtime_error("Error configuring tool path planner: "));
    }

    // Tool path modifiers
    tool_path_modifier_loader_widget_->configure(config[TOOL_PATH_MODIFIERS_KEY]);
  }
  catch (const std::exception&)
  {
    // Clear the widgets
    ui_->scroll_area->setWidget(new QWidget(this));
    mesh_modifier_loader_widget_->removeWidgets();
    tool_path_modifier_loader_widget_->removeWidgets();

    throw;
  }
}

void TPPPipelineWidget::save(YAML::Node& config) const
{
  // Mesh modifier
  {
    YAML::Node mm_config;
    mesh_modifier_loader_widget_->save(mm_config);
    config[MESH_MODIFIERS_KEY] = mm_config;
  }

  // Tool path planner
  {
    auto tpp_widget = dynamic_cast<const ToolPathPlannerWidget*>(ui_->scroll_area->widget());
    if (tpp_widget)
    {
      YAML::Node tpp_config;
      tpp_config["name"] = ui_->group_box_tpp->title().toStdString();
      tpp_widget->save(tpp_config);
      config[TOOL_PATH_PLANNER_KEY] = tpp_config;
    }
  }

  // Tool path modifiers
  {
    YAML::Node tpm_config;
    tool_path_modifier_loader_widget_->save(tpm_config);
    config[TOOL_PATH_MODIFIERS_KEY] = tpm_config;
  }
}

ToolPathPlannerPipeline TPPPipelineWidget::createPipeline() const
{
  auto widget_tpp = dynamic_cast<ToolPathPlannerWidget*>(ui_->scroll_area->widget());
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
        convert<ToolPathModifier>(tool_path_modifier_loader_widget_->getWidgets());

    if (tool_path_mods.empty())
      tool_path_mod = std::make_unique<const ToolPathModifier>();
    else
      tool_path_mod = std::make_unique<const CompoundModifier>(std::move(tool_path_mods));
  }

  return ToolPathPlannerPipeline(std::move(mesh_mod), std::move(tpp), std::move(tool_path_mod));
}

}  // namespace noether
