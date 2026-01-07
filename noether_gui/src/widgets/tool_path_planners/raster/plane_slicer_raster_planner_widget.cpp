#include <noether_gui/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.h>
#include "ui_plane_slicer_raster_planner_widget.h"

#include <noether_tpp/serialization.h>
#include <QGroupBox>

static const std::string BIDIRECTIONAL_KEY = "bidirectional";

namespace noether
{
PlaneSlicerRasterPlannerWidget::PlaneSlicerRasterPlannerWidget(std::shared_ptr<const WidgetFactory> factory,
                                                               QWidget* parent)
  : RasterPlannerWidget(factory, parent), ui_plane_slicer_(new Ui::PlaneSlicerRasterPlanner())
{
  auto widget = new QGroupBox(this);
  ui_plane_slicer_->setupUi(widget);
  layout()->addWidget(widget);
}

void PlaneSlicerRasterPlannerWidget::configure(const YAML::Node& config)
{
  RasterPlannerWidget::configure(config);

  // Optionally get bidirectional parameter to maintain backwards compatibility
  try
  {
    ui_plane_slicer_->check_box_bidirectional->setChecked(YAML::getMember<bool>(config, BIDIRECTIONAL_KEY));
  }
  catch (const std::exception& ex)
  {
  }
}

void PlaneSlicerRasterPlannerWidget::save(YAML::Node& config) const
{
  config["name"] = "PlaneSlicer";
  RasterPlannerWidget::save(config);
  config[BIDIRECTIONAL_KEY] = ui_plane_slicer_->check_box_bidirectional->isChecked();
}

}  // namespace noether
