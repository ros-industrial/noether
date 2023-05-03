#include <noether_gui/widgets/raster_planner_widget.h>
#include "ui_raster_planner_widget.h"
#include <noether_gui/plugin_interface.h>
#include <noether_gui/widgets/collapsible_area_widget.h>
#include <noether_gui/utils.h>

#include <QMessageBox>

namespace noether
{
RasterPlannerWidget::RasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader,
                                         QWidget* parent,
                                         const double line_spacing,
                                         const double point_spacing,
                                         const double min_hole_size)
  : ToolPathPlannerWidget(parent), loader_(std::move(loader)), ui_(new Ui::RasterPlanner())
{
  ui_->setupUi(this);

  // Populate the combo boxes
  ui_->combo_box_dir_gen->addItems(getAvailablePlugins<DirectionGeneratorWidgetPlugin>(loader_));
  ui_->combo_box_origin_gen->addItems(getAvailablePlugins<OriginGeneratorWidgetPlugin>(loader_));

  ui_->double_spin_box_line_spacing->setValue(line_spacing);
  ui_->double_spin_box_point_spacing->setValue(point_spacing);
  ui_->double_spin_box_minimum_hole_size->setValue(min_hole_size);

  connect(ui_->combo_box_dir_gen, &QComboBox::currentTextChanged, [this](const QString& text) {
    if (text.isEmpty())
    {
      overwriteWidget(ui_->group_box_dir_gen->layout(), ui_->widget_dir_gen, new QWidget(this));
    }
    else
    {
      try
      {
        auto plugin = loader_.createInstance<DirectionGeneratorWidgetPlugin>(text.toStdString());

        auto collapsible_area = new CollapsibleArea(text, this);
        collapsible_area->setWidget(plugin->create(this));

        overwriteWidget(ui_->group_box_dir_gen->layout(), ui_->widget_dir_gen, collapsible_area);
      }
      catch (const std::exception& ex)
      {
        QMessageBox::warning(this, "Direction Generator Error", QString::fromStdString(ex.what()));
      }
    }
  });

  connect(ui_->combo_box_origin_gen, &QComboBox::currentTextChanged, [this](const QString& text) {
    if (text.isEmpty())
    {
      overwriteWidget(ui_->group_box_origin_gen->layout(), ui_->widget_origin_gen, new QWidget(this));
    }
    else
    {
      try
      {
        auto plugin = loader_.createInstance<OriginGeneratorWidgetPlugin>(text.toStdString());

        auto collapsible_area = new CollapsibleArea(text, this);
        collapsible_area->setWidget(plugin->create(this));

        overwriteWidget(ui_->group_box_origin_gen->layout(), ui_->widget_origin_gen, collapsible_area);
      }
      catch (const std::exception& ex)
      {
        QMessageBox::warning(this, "Origin Generator Error", QString::fromStdString(ex.what()));
      }
    }
  });
}

DirectionGeneratorWidget* RasterPlannerWidget::getDirectionGeneratorWidget() const
{
  auto collapsible_area = dynamic_cast<CollapsibleArea*>(ui_->widget_dir_gen);
  if (!collapsible_area)
    throw std::runtime_error("No direction generator specified");
  return collapsible_area->getWidget<DirectionGeneratorWidget>();
}

OriginGeneratorWidget* RasterPlannerWidget::getOriginGeneratorWidget() const
{
  auto collapsible_area = dynamic_cast<CollapsibleArea*>(ui_->widget_origin_gen);
  if (!collapsible_area)
    throw std::runtime_error("No origin generator specified");
  return collapsible_area->getWidget<OriginGeneratorWidget>();
}

}  // namespace noether
