#include <noether_gui/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.h>
#include "ui_raster_planner_widget.h"

#include <noether_tpp/serialization.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QCheckBox>

namespace noether
{
PlaneSlicerRasterPlannerWidget::PlaneSlicerRasterPlannerWidget(std::shared_ptr<const GuiFactory> factory,
                                                               QWidget* parent)
  : RasterPlannerWidget(factory, parent)
  , search_radius_(new DistanceDoubleSpinBox(this))
  , min_segment_size_(new DistanceDoubleSpinBox(this))
  , bidirectional_(new QCheckBox(this))
{
  // Search radius
  search_radius_->setMinimum(0.0);
  search_radius_->setSingleStep(0.1);
  search_radius_->setValue(0.1);
  search_radius_->setDecimals(3);
  auto search_radius_label = new QLabel("Normal Radius", this);
  search_radius_label->setToolTip("Radius with which to estimate mesh normals");
  ui_->form_layout->addRow(search_radius_label, search_radius_);

  // Min segment length
  min_segment_size_->setMinimum(0.0);
  min_segment_size_->setSingleStep(0.1);
  min_segment_size_->setValue(0.1);
  min_segment_size_->setDecimals(3);
  auto min_segment_label = new QLabel("Min Segment Length", this);
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
  search_radius_->setValue(YAML::getMember<double>(config, "search_radius"));
  min_segment_size_->setValue(YAML::getMember<double>(config, "min_segment_size"));

  // Optionally get bidirectional parameter to maintain backwards compatibility
  try
  {
    bidirectional_->setChecked(YAML::getMember<bool>(config, "bidirectional"));
  }
  catch (const std::exception& ex)
  {
  }
}

void PlaneSlicerRasterPlannerWidget::save(YAML::Node& config) const
{
  RasterPlannerWidget::save(config);
  config["name"] = "PlaneSlicer";
  config["search_radius"] = search_radius_->value();
  config["min_segment_size"] = min_segment_size_->value();
  config["bidirectional"] = bidirectional_->isChecked();
}

}  // namespace noether
