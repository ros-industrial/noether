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
PlaneSlicerRasterPlannerWidget::PlaneSlicerRasterPlannerWidget(std::shared_ptr<const WidgetFactory> factory,
                                                               QWidget* parent)
  : RasterPlannerWidget(factory, parent)
  , min_segment_size_(new DistanceDoubleSpinBox(this))
  , bidirectional_(new QCheckBox(this))
{
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
  config["name"] = "PlaneSlicer";
  RasterPlannerWidget::save(config);
  config["min_segment_size"] = min_segment_size_->value();
  config["bidirectional"] = bidirectional_->isChecked();
}

}  // namespace noether
