#include <noether_gui/widgets/tool_path_planners/flat_plane_toolpath_planner_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/flat_plane_toolpath_planner.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include "ui_vector2d_editor_widget.h"
#include "ui_isometry3d_editor_widget.h"

static const std::string ORIGIN_PX_KEY = "px";
static const std::string ORIGIN_PY_KEY = "py";
static const std::string ORIGIN_PZ_KEY = "pz";
static const std::string ORIGIN_QX_KEY = "qx";
static const std::string ORIGIN_QY_KEY = "qy";
static const std::string ORIGIN_QZ_KEY = "qz";
static const std::string ORIGIN_QW_KEY = "qw";

static const std::string PLANE_X_DIM_KEY = "plane_x_dim";
static const std::string PLANE_Y_DIM_KEY = "plane_y_dim";
static const std::string X_SPACING_KEY = "x_spacing";
static const std::string Y_SPACING_KEY = "y_spacing";

namespace noether
{
FlatPlaneToolPathPlannerWidget::FlatPlaneToolPathPlannerWidget(QWidget* parent)
  : ToolPathPlannerWidget(parent)
  , origin_ui_(new Ui::Isometry3dEditor())
  , plane_dim_ui_(new Ui::Vector2dEditor())
  , spacing_dim_ui_(new Ui::Vector2dEditor())
{
  // Create a vertical layout for the entire widget
  auto mainLayout = new QVBoxLayout(this);

  origin_ui_->setupUi(this);
  origin_ui_->group_box->setTitle("Offset (position in meters, rotation in radians)");

  plane_dim_ui_->setupUi(this);
  plane_dim_ui_->group_box->setTitle("Plane Dimensions (m)");

  spacing_dim_ui_->setupUi(this);
  spacing_dim_ui_->group_box->setTitle("Plane Spacing (m)");

  // Create a container widget to hold the layouts
  auto containerWidget = new QWidget;

  // Create a layout for the container widget
  auto containerLayout = new QVBoxLayout(containerWidget);

  // Create a horizontal layout for the spin box and label
  auto spinBoxLayout = new QHBoxLayout;

  // Add the horizontal layout to the container layout
  containerLayout->addLayout(spinBoxLayout);

  // Set the container widget as the layout for the Isometry3dEditor and Vector2dEditor widgets
  origin_ui_->group_box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  containerLayout->addWidget(origin_ui_->group_box);

  plane_dim_ui_->group_box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  containerLayout->addWidget(plane_dim_ui_->group_box);

  spacing_dim_ui_->group_box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  containerLayout->addWidget(spacing_dim_ui_->group_box);

  // Add the container widget to the main layout
  mainLayout->addWidget(containerWidget);

  // Set the main layout for the entire widget
  setLayout(mainLayout);
}

ToolPathPlanner::ConstPtr FlatPlaneToolPathPlannerWidget::create() const
{
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  // Create a quaternion from roll, pitch, yaw
  Eigen::Quaterniond q(origin_ui_->double_spin_box_qw->value(),
                       origin_ui_->double_spin_box_qx->value(),
                       origin_ui_->double_spin_box_qy->value(),
                       origin_ui_->double_spin_box_qz->value());

  // Set the translation
  origin.translation() = Eigen::Vector3d(origin_ui_->double_spin_box_px->value(),
                                         origin_ui_->double_spin_box_py->value(),
                                         origin_ui_->double_spin_box_pz->value());

  // Set the rotation
  origin.rotate(q);

  Eigen::Vector2d plane_dim(plane_dim_ui_->double_spin_box_x->value(), plane_dim_ui_->double_spin_box_y->value());
  Eigen::Vector2d spacing(spacing_dim_ui_->double_spin_box_x->value(), spacing_dim_ui_->double_spin_box_y->value());
  return std::make_unique<FlatPlaneToolPathPlanner>(plane_dim, spacing, origin);
}

void FlatPlaneToolPathPlannerWidget::configure(const YAML::Node& config)
{
  origin_ui_->double_spin_box_px->setValue(getEntry<double>(config, ORIGIN_PX_KEY));
  origin_ui_->double_spin_box_py->setValue(getEntry<double>(config, ORIGIN_PY_KEY));
  origin_ui_->double_spin_box_pz->setValue(getEntry<double>(config, ORIGIN_PZ_KEY));
  origin_ui_->double_spin_box_qx->setValue(getEntry<double>(config, ORIGIN_QX_KEY));
  origin_ui_->double_spin_box_qy->setValue(getEntry<double>(config, ORIGIN_QY_KEY));
  origin_ui_->double_spin_box_qz->setValue(getEntry<double>(config, ORIGIN_QZ_KEY));
  origin_ui_->double_spin_box_qw->setValue(getEntry<double>(config, ORIGIN_QW_KEY));

  Eigen::Vector2d plane_dim(getEntry<double>(config, PLANE_X_DIM_KEY), getEntry<double>(config, PLANE_Y_DIM_KEY));
  plane_dim_ui_->double_spin_box_x->setValue(getEntry<double>(config, PLANE_X_DIM_KEY));
  plane_dim_ui_->double_spin_box_y->setValue(getEntry<double>(config, PLANE_Y_DIM_KEY));

  Eigen::Vector2d spacing(getEntry<double>(config, X_SPACING_KEY), getEntry<double>(config, Y_SPACING_KEY));
  spacing_dim_ui_->double_spin_box_x->setValue(getEntry<double>(config, X_SPACING_KEY));
  spacing_dim_ui_->double_spin_box_y->setValue(getEntry<double>(config, Y_SPACING_KEY));
}

void FlatPlaneToolPathPlannerWidget::save(YAML::Node& config) const
{
  config[ORIGIN_PX_KEY] = origin_ui_->double_spin_box_px->value();
  config[ORIGIN_PY_KEY] = origin_ui_->double_spin_box_py->value();
  config[ORIGIN_PZ_KEY] = origin_ui_->double_spin_box_pz->value();
  config[ORIGIN_QX_KEY] = origin_ui_->double_spin_box_qx->value();
  config[ORIGIN_QY_KEY] = origin_ui_->double_spin_box_qy->value();
  config[ORIGIN_QZ_KEY] = origin_ui_->double_spin_box_qz->value();
  config[ORIGIN_QW_KEY] = origin_ui_->double_spin_box_qw->value();

  config[PLANE_X_DIM_KEY] = plane_dim_ui_->double_spin_box_x->value();
  config[PLANE_Y_DIM_KEY] = plane_dim_ui_->double_spin_box_y->value();

  config[X_SPACING_KEY] = spacing_dim_ui_->double_spin_box_x->value();
  config[Y_SPACING_KEY] = spacing_dim_ui_->double_spin_box_y->value();
}

}  // namespace noether
