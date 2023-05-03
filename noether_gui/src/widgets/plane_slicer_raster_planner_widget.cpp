#include <noether_gui/widgets/plane_slicer_raster_planner_widget.h>
#include "ui_raster_planner_widget.h"
#include <noether_gui/widgets/collapsible_area_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <yaml-cpp/yaml.h>

namespace noether
{
PlaneSlicerRasterPlannerWidget::PlaneSlicerRasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader,
                                                               QWidget* parent)
  : RasterPlannerWidget(std::move(loader), parent)
  , search_radius_(new QDoubleSpinBox(this))
  , min_segment_size_(new QDoubleSpinBox(this))
{
  auto layout = new QFormLayout();

  // Search radius
  search_radius_->setMinimum(0.0);
  search_radius_->setSingleStep(0.1);
  search_radius_->setValue(0.1);
  search_radius_->setDecimals(3);
  layout->addRow(new QLabel("Search radius (m)", this), search_radius_);

  // Min segment length
  min_segment_size_->setMinimum(0.0);
  min_segment_size_->setSingleStep(0.1);
  min_segment_size_->setValue(0.1);
  min_segment_size_->setDecimals(3);
  layout->addRow(new QLabel("Min. segment length", this), min_segment_size_);

  auto widget = new QWidget(this);
  widget->setLayout(layout);

  auto collapsible_area = new CollapsibleArea("PlaneSlicerRasterPlanner", this);
  collapsible_area->setWidget(widget);

  ui_->group_box_raster_planner->layout()->addWidget(collapsible_area);
}

void PlaneSlicerRasterPlannerWidget::fromYAML(const YAML::Node& config)
{
  RasterPlannerWidget::fromYAML(config);
  search_radius_->setValue(getEntry<double>(config, "search_radius"));
  min_segment_size_->setValue(getEntry<double>(config, "min_segment_size"));
}

ToolPathPlanner::ConstPtr PlaneSlicerRasterPlannerWidget::create() const
{
  DirectionGeneratorWidget* dir_gen_widget = getDirectionGeneratorWidget();
  OriginGeneratorWidget* origin_gen_widget = getOriginGeneratorWidget();

  auto planner = std::make_unique<PlaneSlicerRasterPlanner>(dir_gen_widget->create(), origin_gen_widget->create());
  planner->setLineSpacing(ui_->double_spin_box_line_spacing->value());
  planner->setPointSpacing(ui_->double_spin_box_point_spacing->value());
  planner->setMinHoleSize(ui_->double_spin_box_minimum_hole_size->value());
  planner->setSearchRadius(search_radius_->value());
  planner->setMinSegmentSize(min_segment_size_->value());

  return planner;
}

}  // namespace noether
