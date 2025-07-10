#include <noether_gui/widgets/tool_path_planners/raster/cross_hatch_plane_slicer_raster_planner_widget.h>
#include "ui_raster_planner_widget.h"
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/multi_tool_path_planner.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/pca_rotated_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <QDoubleSpinBox>
#include <yaml-cpp/yaml.h>

namespace noether
{
CrossHatchPlaneSlicerRasterPlannerWidget::CrossHatchPlaneSlicerRasterPlannerWidget(
    boost_plugin_loader::PluginLoader&& loader,
    QWidget* parent)
  : PlaneSlicerRasterPlannerWidget(std::move(loader), parent), cross_hatch_angle_(new QDoubleSpinBox(this))
{
  // Search radius
  cross_hatch_angle_->setMinimum(0.0);
  cross_hatch_angle_->setSingleStep(1.0);
  cross_hatch_angle_->setValue(90.0);
  cross_hatch_angle_->setDecimals(3);
  ui_->form_layout->addRow(new QLabel("Cross Hatch Angle (deg)", this), cross_hatch_angle_);
}

ToolPathPlanner::ConstPtr CrossHatchPlaneSlicerRasterPlannerWidget::create() const
{
  // Create the nominal tool path planner
  auto nominal_tpp = PlaneSlicerRasterPlannerWidget::create();

  // Make another PlaneSliceRasterPlanner with a PCA-rotated direction generator
  OriginGenerator::ConstPtr nominal_origin_gen = getOriginGeneratorWidget()->create();
  DirectionGenerator::ConstPtr nominal_dir_gen = getDirectionGeneratorWidget()->create();
  auto dir_gen = std::make_unique<PCARotatedDirectionGenerator>(std::move(nominal_dir_gen),
                                                                cross_hatch_angle_->value() * M_PI / 180.0);
  auto cross_hatch_tpp = std::make_unique<PlaneSlicerRasterPlanner>(std::move(dir_gen), std::move(nominal_origin_gen));

  // Configure the tool path planner with the UI
  cross_hatch_tpp->setLineSpacing(ui_->double_spin_box_line_spacing->value());
  cross_hatch_tpp->setPointSpacing(ui_->double_spin_box_point_spacing->value());
  cross_hatch_tpp->setMinHoleSize(ui_->double_spin_box_minimum_hole_size->value());
  cross_hatch_tpp->setSearchRadius(search_radius_->value());
  cross_hatch_tpp->setMinSegmentSize(min_segment_size_->value());

  // Return a multi tool path planner
  std::vector<ToolPathPlanner::ConstPtr> planners;
  planners.emplace_back(std::move(nominal_tpp));
  planners.emplace_back(std::move(cross_hatch_tpp));
  return std::make_unique<MultiToolPathPlanner>(std::move(planners));
}

void CrossHatchPlaneSlicerRasterPlannerWidget::configure(const YAML::Node& config)
{
  PlaneSlicerRasterPlannerWidget::configure(config);
  cross_hatch_angle_->setValue(getEntry<double>(config, "cross_hatch_angle"));
}

void CrossHatchPlaneSlicerRasterPlannerWidget::save(YAML::Node& config) const
{
  PlaneSlicerRasterPlannerWidget::save(config);
  config["cross_hatch_angle"] = cross_hatch_angle_->value();
}

}  // namespace noether
