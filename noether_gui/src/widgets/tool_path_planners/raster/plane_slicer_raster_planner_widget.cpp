#include <noether_gui/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.h>
#include "ui_plane_slicer_raster_planner_widget.h"

#include <noether_tpp/serialization.h>
#include <QGroupBox>

static const std::string POINT_SPACING_KEY = "point_spacing";
static const std::string MIN_HOLE_SIZE_KEY = "min_hole_size";
static const std::string SEARCH_RADIUS_KEY = "search_radius";
static const std::string MIN_SEGMENT_SIZE_KEY = "min_segment_size";
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

  ui_plane_slicer_->double_spin_box_point_spacing->setValue(YAML::getMember<double>(config, POINT_SPACING_KEY));
  ui_plane_slicer_->double_spin_box_minimum_hole_size->setValue(YAML::getMember<double>(config, MIN_HOLE_SIZE_KEY));
  ui_plane_slicer_->double_spin_box_search_radius->setValue(YAML::getMember<double>(config, SEARCH_RADIUS_KEY));
  ui_plane_slicer_->double_spin_box_min_segment_size->setValue(YAML::getMember<double>(config, MIN_SEGMENT_SIZE_KEY));

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
  config[POINT_SPACING_KEY] = ui_plane_slicer_->double_spin_box_point_spacing->value();
  config[MIN_HOLE_SIZE_KEY] = ui_plane_slicer_->double_spin_box_minimum_hole_size->value();
  config[SEARCH_RADIUS_KEY] = ui_plane_slicer_->double_spin_box_search_radius->value();
  config[MIN_SEGMENT_SIZE_KEY] = ui_plane_slicer_->double_spin_box_min_segment_size->value();
  config[BIDIRECTIONAL_KEY] = ui_plane_slicer_->check_box_bidirectional->isChecked();
}

}  // namespace noether
