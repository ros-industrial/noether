#include <noether_gui/widgets/mesh_modifiers/ransac_cylinder_fit_modifier_widget.h>
#include <noether_gui/widgets/angle_double_spin_box.h>
#include <noether_gui/widgets/distance_double_spin_box.h>
#include "../ui_vector3d_editor_widget.h"
#include "ui_ransac_cylinder_fit_modifier_widget.h"
#include "ui_ransac_primitive_fit_modifier_widget.h"

#include <noether_tpp/serialization.h>
#include <QCheckBox>
#include <QSpinBox>

namespace noether
{
RansacCylinderProjectionMeshModifierWidget::RansacCylinderProjectionMeshModifierWidget(QWidget* parent)
  : RansacPrimitiveFitMeshModifierWidget(parent)
  , model_ui_(new Ui::RansacCylinderProjection())
  , axis_(new Ui::Vector3dEditor())
{
  // Add a tab for the SAC parameters
  {
    auto w = new QWidget(this);
    ui_->tab_widget->addTab(w, "Cylinder");
    model_ui_->setupUi(w);
  }

  // Add a tab for the axis parameters
  {
    auto w = new QWidget(this);
    ui_->tab_widget->addTab(w, "Axis");
    axis_->setupUi(w);
    axis_->group_box->setTitle("Axis");
    axis_->double_spin_box_z->setValue(1.0);
  }
}

void RansacCylinderProjectionMeshModifierWidget::configure(const YAML::Node& config)
{
  RansacPrimitiveFitMeshModifierWidget::configure(config);

  model_ui_->radius_min->setValue(YAML::getMember<double>(config, "min_radius"));
  model_ui_->radius_max->setValue(YAML::getMember<double>(config, "max_radius"));

  auto axis = YAML::getMember<Eigen::Vector3f>(config, "axis");
  axis_->double_spin_box_x->setValue(axis.x());
  axis_->double_spin_box_y->setValue(axis.y());
  axis_->double_spin_box_z->setValue(axis.z());

  model_ui_->axis_threshold->setValue(YAML::getMember<double>(config, "axis_threshold"));
  model_ui_->normal_distance_weight->setValue(YAML::getMember<double>(config, "normal_distance_weight"));
}

void RansacCylinderProjectionMeshModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "RansacCylinderProjection";

  RansacPrimitiveFitMeshModifierWidget::save(config);

  config["min_radius"] = model_ui_->radius_min->value();
  config["max_radius"] = model_ui_->radius_max->value();

  Eigen::Vector3f axis = Eigen::Vector3d{ axis_->double_spin_box_x->value(),
                                          axis_->double_spin_box_y->value(),
                                          axis_->double_spin_box_z->value() }
                             .cast<float>();
  config["axis"] = axis;

  config["axis_threshold"] = model_ui_->axis_threshold->value();
  config["normal_distance_weight"] = model_ui_->normal_distance_weight->value();
}

RansacCylinderFitMeshModifierWidget::RansacCylinderFitMeshModifierWidget(QWidget* parent)
  : RansacCylinderProjectionMeshModifierWidget(parent)
  , resolution_(new QSpinBox(this))
  , include_caps_(new QCheckBox(this))
  , uniform_triangles_(new QCheckBox(this))
{
  // Add a tab for the primitive parameters
  auto w = new QWidget(this);
  auto layout = new QVBoxLayout(w);

  // Resolution
  {
    resolution_->setMinimum(6);
    resolution_->setMaximum(1e6);
    resolution_->setSingleStep(1);
    resolution_->setValue(50);
    resolution_->setToolTip("Number of vertices around the perimiter of the cylinder primitive");

    auto form_layout = new QFormLayout(w);
    form_layout->addRow("Resolution", resolution_);
    layout->addLayout(form_layout);
  }

  // Include caps
  {
    include_caps_->setChecked(false);
    include_caps_->setText("Include caps");
    include_caps_->setToolTip("Include the caps of the cylinder primitive");
    layout->addWidget(include_caps_);
  }

  // Uniform triangles
  {
    uniform_triangles_->setChecked(true);
    uniform_triangles_->setText("Uniform triangles");
    uniform_triangles_->setToolTip("Generate cylinder body with uniform shaped triangles");
    layout->addWidget(uniform_triangles_);
  }

  ui_->tab_widget->addTab(w, "Primitive");
}

void RansacCylinderFitMeshModifierWidget::configure(const YAML::Node& config)
{
  RansacCylinderProjectionMeshModifierWidget::configure(config);
  resolution_->setValue(YAML::getMember<int>(config, "resolution"));
  include_caps_->setChecked(YAML::getMember<bool>(config, "include_caps"));
  uniform_triangles_->setChecked(YAML::getMember<bool>(config, "uniform_triangles"));
}

void RansacCylinderFitMeshModifierWidget::save(YAML::Node& config) const
{
  RansacCylinderProjectionMeshModifierWidget::save(config);
  config["name"] = "RansacCylinderFit";
  config["resolution"] = resolution_->value();
  config["include_caps"] = include_caps_->isChecked();
  config["uniform_triangles"] = uniform_triangles_->isChecked();
}

}  // namespace noether
