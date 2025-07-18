#include <noether_gui/widgets/tool_path_planners/raster/cross_hatch_plane_slicer_raster_planner_widget.h>
#include <noether_gui/widgets/angle_double_spin_box.h>
#include "ui_raster_planner_widget.h"

#include <noether_tpp/serialization.h>
#include <QDoubleSpinBox>

namespace noether
{
CrossHatchPlaneSlicerRasterPlannerWidget::CrossHatchPlaneSlicerRasterPlannerWidget(
    std::shared_ptr<const GuiFactory> factory,
    QWidget* parent)
  : PlaneSlicerRasterPlannerWidget(factory, parent), cross_hatch_angle_(new AngleDoubleSpinBox(this))
{
  // Search radius
  cross_hatch_angle_->setMinimum(0.0);
  cross_hatch_angle_->setValue(M_PI_2);
  cross_hatch_angle_->setDecimals(3);
  ui_->form_layout->addRow(new QLabel("Cross Hatch Angle", this), cross_hatch_angle_);
}

void CrossHatchPlaneSlicerRasterPlannerWidget::configure(const YAML::Node& config)
{
  YAML::Node planners_config = YAML::getMember<YAML::Node>(config, "planners");
  if (planners_config.size() != 2)
    throw std::runtime_error("Cross hatch plane slicer raster planner configuration must specify 2 planners");

  // Configure the plane slicer parameters from the first planner configuration
  PlaneSlicerRasterPlannerWidget::configure(planners_config[0]);

  // Extract the cross-hatch angle from the second planner configuration
  auto dir_gen_config = YAML::getMember<YAML::Node>(planners_config[1], "direction_generator");
  const double rotation_offset = YAML::getMember<double>(dir_gen_config, "rotation_offset");
  cross_hatch_angle_->setValue(rotation_offset);
}

void CrossHatchPlaneSlicerRasterPlannerWidget::save(YAML::Node& config) const
{
  config["name"] = "Multi";
  config["gui_plugin_name"] = "CrossHatchPlaneSlicer";

  // Add the nominal planner configuration
  YAML::Node nominal_planner_config;
  PlaneSlicerRasterPlannerWidget::save(nominal_planner_config);
  config["planners"].push_back(nominal_planner_config);

  // Add the cross-hatch planner configuration
  YAML::Node cross_hatch_planner_config = YAML::Clone(nominal_planner_config);

  // Replace the direction generator with a rotate principal axis direction generator
  YAML::Node pca_dir_gen_config;
  pca_dir_gen_config["name"] = "PCARotated";
  pca_dir_gen_config["direction_generator"] = YAML::Clone(nominal_planner_config["direction_generator"]);
  pca_dir_gen_config["rotation_offset"] = cross_hatch_angle_->value();
  cross_hatch_planner_config["direction_generator"] = pca_dir_gen_config;
  config["planners"].push_back(cross_hatch_planner_config);
}

}  // namespace noether
