#include <noether_gui/widgets/mesh_modifiers/cylinder_projection_modifier_widget.h>
#include <noether_gui/widgets/angle_double_spin_box.h>
#include <noether_gui/widgets/distance_double_spin_box.h>
#include "../ui_vector3d_editor_widget.h"
#include "ui_cylinder_projection_modifier_widget.h"

#include <noether_tpp/serialization.h>

namespace noether
{
CylinderProjectionModifierWidget::CylinderProjectionModifierWidget(QWidget* parent)
  : ui_(new Ui::CylinderSegmentation()), axis_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);

  // Axis
  axis_->setupUi(ui_->axis_widget);
  axis_->group_box->setTitle("Axis");
  axis_->double_spin_box_z->setValue(1.0);
}

void CylinderProjectionModifierWidget::configure(const YAML::Node& config)
{
  ui_->radius_min->setValue(YAML::getMember<double>(config, "min_radius"));
  ui_->radius_max->setValue(YAML::getMember<double>(config, "max_radius"));
  ui_->distance_threshold->setValue(YAML::getMember<double>(config, "distance_threshold"));

  auto axis = YAML::getMember<Eigen::Vector3f>(config, "axis");
  axis_->double_spin_box_x->setValue(axis.x());
  axis_->double_spin_box_y->setValue(axis.y());
  axis_->double_spin_box_z->setValue(axis.z());

  ui_->axis_threshold->setValue(YAML::getMember<double>(config, "axis_threshold"));
  ui_->normal_distance_weight->setValue(YAML::getMember<double>(config, "normal_distance_weight"));
  ui_->min_vertices->setValue(YAML::getMember<int>(config, "min_vertices"));
  ui_->max_cylinders->setValue(YAML::getMember<int>(config, "max_cylinders"));
  ui_->max_iterations->setValue(YAML::getMember<int>(config, "max_iterations"));
}

void CylinderProjectionModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "CylinderSegmentation";
  config["min_radius"] = ui_->radius_min->value();
  config["max_radius"] = ui_->radius_max->value();
  config["distance_threshold"] = ui_->distance_threshold->value();

  Eigen::Vector3f axis = Eigen::Vector3d{ axis_->double_spin_box_x->value(),
                                          axis_->double_spin_box_y->value(),
                                          axis_->double_spin_box_z->value() }
                             .cast<float>();
  config["axis"] = axis;

  config["axis_threshold"] = ui_->axis_threshold->value();
  config["normal_distance_weight"] = ui_->normal_distance_weight->value();
  config["min_vertices"] = ui_->min_vertices->value();
  config["max_cylinders"] = ui_->max_cylinders->value();
  config["max_iterations"] = ui_->max_iterations->value();
}

}  // namespace noether
