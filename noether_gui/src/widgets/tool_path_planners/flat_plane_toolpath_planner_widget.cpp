#include <noether_gui/widgets/distance_double_spin_box.h>
#include <noether_gui/widgets/tool_path_planners/flat_plane_toolpath_planner_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/flat_plane_toolpath_planner.h>
#include <noether_tpp/serialization.h>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>

namespace noether
{
FlatPlaneToolPathPlannerWidget::FlatPlaneToolPathPlannerWidget(QWidget* parent) : ToolPathPlannerWidget(parent)
{
  // Create a vertical layout for the entire widget
  auto main_layout = new QVBoxLayout(this);

  // Create Widget to Store Plane Parameters
  auto plane_params_widget = new QGroupBox("Plane Parameters");
  auto plane_params_layout = new QFormLayout;

  plane_x_dim_spinbox_ = new noether::DistanceDoubleSpinBox;
  plane_x_dim_spinbox_->setMinimum(0.001);
  plane_x_dim_spinbox_->setValue(1.0);

  plane_y_dim_spinbox_ = new noether::DistanceDoubleSpinBox;
  plane_y_dim_spinbox_->setMinimum(0.001);
  plane_y_dim_spinbox_->setValue(1.0);

  plane_x_spacing_spinbox_ = new noether::DistanceDoubleSpinBox;
  plane_x_spacing_spinbox_->setMinimum(0.001);
  plane_x_spacing_spinbox_->setValue(0.1);

  plane_y_spacing_spinbox_ = new noether::DistanceDoubleSpinBox;
  plane_y_spacing_spinbox_->setMinimum(0.001);
  plane_y_spacing_spinbox_->setValue(0.1);

  plane_params_layout->addRow("X Dimension", plane_x_dim_spinbox_);
  plane_params_layout->addRow("Y Dimension", plane_y_dim_spinbox_);
  plane_params_layout->addRow("Waypoint X Spacing", plane_x_spacing_spinbox_);
  plane_params_layout->addRow("Waypoint Y Spacing", plane_y_spacing_spinbox_);

  plane_params_widget->setLayout(plane_params_layout);

  plane_params_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  main_layout->addWidget(plane_params_widget);

  // Set the main layout for the entire widget
  setLayout(main_layout);
}

// ToolPathPlanner::ConstPtr FlatPlaneToolPathPlannerWidget::create() const
// {
//   Eigen::Vector2d plane_dim(plane_x_dim_spinbox_->value(), plane_y_dim_spinbox_->value());
//   Eigen::Vector2d spacing(plane_x_spacing_spinbox_->value(), plane_y_spacing_spinbox_->value());
//   return std::make_unique<FlatPlaneToolPathPlanner>(plane_dim, spacing);
// }

void FlatPlaneToolPathPlannerWidget::configure(const YAML::Node& config)
{
  plane_x_dim_spinbox_->setValue(YAML::getMember<double>(config, "plane_x_dim"));
  plane_y_dim_spinbox_->setValue(YAML::getMember<double>(config, "plane_y_dim"));
  plane_x_spacing_spinbox_->setValue(YAML::getMember<double>(config, "plane_x_spacing"));
  plane_y_spacing_spinbox_->setValue(YAML::getMember<double>(config, "plane_y_spacing"));
}

void FlatPlaneToolPathPlannerWidget::save(YAML::Node& config) const
{
  config["name"] = "FlatPlane";
  config["plane_x_dim"] = plane_x_dim_spinbox_->value();
  config["plane_y_dim"] = plane_x_dim_spinbox_->value();
  config["plane_x_spacing"] = plane_x_spacing_spinbox_->value();
  config["plane_y_spacing"] = plane_y_spacing_spinbox_->value();
}

}  // namespace noether
