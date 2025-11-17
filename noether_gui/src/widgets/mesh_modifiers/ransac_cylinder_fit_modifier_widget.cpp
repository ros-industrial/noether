#include <noether_gui/widgets/mesh_modifiers/ransac_cylinder_fit_modifier_widget.h>
#include <noether_gui/widgets/angle_double_spin_box.h>
#include <noether_gui/widgets/distance_double_spin_box.h>
#include "../ui_vector3d_editor_widget.h"
#include "ui_ransac_cylinder_fit_modifier_widget.h"
#include "ui_ransac_primitive_fit_modifier_widget.h"

#include <noether_tpp/serialization.h>

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

void RansacCylinderFitMeshModifierWidget::save(YAML::Node& config) const
{
  RansacCylinderProjectionMeshModifierWidget::save(config);
  config["name"] = "RansacCylinderFit";
}

}  // namespace noether
