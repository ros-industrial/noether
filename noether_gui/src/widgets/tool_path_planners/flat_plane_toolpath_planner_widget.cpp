#include <noether_gui/widgets/distance_double_spin_box.h>
#include <noether_gui/widgets/tool_path_planners/flat_plane_toolpath_planner_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/flat_plane_toolpath_planner.h>
#include <noether_tpp/serialization.h>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>

static const std::string X_DIM_KEY = "x_dim";
static const std::string Y_DIM_KEY = "y_dim";
static const std::string X_SPACING_KEY = "x_spacing";
static const std::string Y_SPACING_KEY = "y_spacing";

namespace noether
{
FlatPlaneToolPathPlannerWidget::FlatPlaneToolPathPlannerWidget(QWidget* parent)
  : BaseWidget(parent)
  , x_dim_spinbox_(new DistanceDoubleSpinBox(this))
  , y_dim_spinbox_(new DistanceDoubleSpinBox(this))
  , x_spacing_spinbox_(new DistanceDoubleSpinBox(this))
  , y_spacing_spinbox_(new DistanceDoubleSpinBox(this))
{
  // Create a vertical layout for the entire widget
  auto main_layout = new QVBoxLayout(this);

  // Create Widget to Store Plane Parameters
  auto plane_params_widget = new QGroupBox("Plane Parameters");
  auto plane_params_layout = new QFormLayout;

  x_dim_spinbox_->setMinimum(0.001);
  x_dim_spinbox_->setValue(1.0);

  y_dim_spinbox_->setMinimum(0.001);
  y_dim_spinbox_->setValue(1.0);

  x_spacing_spinbox_->setMinimum(0.001);
  x_spacing_spinbox_->setValue(0.1);

  y_spacing_spinbox_->setMinimum(0.001);
  y_spacing_spinbox_->setValue(0.1);

  plane_params_layout->addRow("X Dimension", x_dim_spinbox_);
  plane_params_layout->addRow("Y Dimension", y_dim_spinbox_);
  plane_params_layout->addRow("Waypoint X Spacing", x_spacing_spinbox_);
  plane_params_layout->addRow("Waypoint Y Spacing", y_spacing_spinbox_);

  plane_params_widget->setLayout(plane_params_layout);

  plane_params_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  main_layout->addWidget(plane_params_widget);

  // Set the main layout for the entire widget
  setLayout(main_layout);
}

void FlatPlaneToolPathPlannerWidget::configure(const YAML::Node& config)
{
  x_dim_spinbox_->setValue(YAML::getMember<double>(config, X_DIM_KEY));
  y_dim_spinbox_->setValue(YAML::getMember<double>(config, Y_DIM_KEY));
  x_spacing_spinbox_->setValue(YAML::getMember<double>(config, X_SPACING_KEY));
  y_spacing_spinbox_->setValue(YAML::getMember<double>(config, Y_SPACING_KEY));
}

void FlatPlaneToolPathPlannerWidget::save(YAML::Node& config) const
{
  config["name"] = "FlatPlane";
  config[X_DIM_KEY] = x_dim_spinbox_->value();
  config[Y_DIM_KEY] = y_dim_spinbox_->value();
  config[X_SPACING_KEY] = x_spacing_spinbox_->value();
  config[Y_SPACING_KEY] = y_spacing_spinbox_->value();
}

}  // namespace noether
