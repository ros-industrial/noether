#include <noether_gui/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.h>
#include "ui_raster_planner_widget.h"
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QCheckBox>
#include <yaml-cpp/yaml.h>

namespace noether
{
PlaneSlicerRasterPlannerWidget::PlaneSlicerRasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader,
                                                               QWidget* parent)
  : RasterPlannerWidget(std::move(loader), parent)
  , search_radius_(new QDoubleSpinBox(this))
  , min_segment_size_(new QDoubleSpinBox(this))
  , bidirectional_(new QCheckBox(this))
{
  // Search radius
  search_radius_->setMinimum(0.0);
  search_radius_->setSingleStep(0.1);
  search_radius_->setValue(0.1);
  search_radius_->setDecimals(3);
  auto search_radius_label = new QLabel("Normal radius (m)", this);
  search_radius_label->setToolTip("Radius with which to estimate mesh normals");
  ui_->form_layout->addRow(search_radius_label, search_radius_);

  // Min segment length
  min_segment_size_->setMinimum(0.0);
  min_segment_size_->setSingleStep(0.1);
  min_segment_size_->setValue(0.1);
  min_segment_size_->setDecimals(3);
  auto min_segment_label = new QLabel("Min. segment length (m)", this);
  min_segment_label->setToolTip("Tool path segments shorter than this length will not be included");
  ui_->form_layout->addRow(min_segment_label, min_segment_size_);

  // Bidirectional checkbox
  bidirectional_->setText("Bidirectional");
  bidirectional_->setChecked(true);
  bidirectional_->setToolTip("Generate rasters in the direction of the cut plane normal and its negation");
  ui_->group_box_raster_planner->layout()->addWidget(bidirectional_);
}

void PlaneSlicerRasterPlannerWidget::configure(const YAML::Node& config)
{
  RasterPlannerWidget::configure(config);
  search_radius_->setValue(getEntry<double>(config, "search_radius"));
  min_segment_size_->setValue(getEntry<double>(config, "min_segment_size"));

  // Optionally get bidirectional parameter to maintain backwards compatibility
  try
  {
    bidirectional_->setChecked(getEntry<bool>(config, "bidirectional"));
  }
  catch (const std::exception& ex)
  {
  }
}

void PlaneSlicerRasterPlannerWidget::save(YAML::Node& config) const
{
  RasterPlannerWidget::save(config);
  config["search_radius"] = search_radius_->value();
  config["min_segment_size"] = min_segment_size_->value();
  config["bidirectional"] = bidirectional_->isChecked();
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
  planner->generateRastersBidirectionally(bidirectional_->isChecked());

  return planner;
}

}  // namespace noether
