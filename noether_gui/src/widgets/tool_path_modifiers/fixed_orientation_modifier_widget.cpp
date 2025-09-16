#include <noether_gui/widgets/tool_path_modifiers/fixed_orientation_modifier_widget.h>
#include "../ui_vector3d_editor_widget.h"

#include <noether_tpp/serialization.h>

namespace noether
{
FixedOrientationModifierWidget::FixedOrientationModifierWidget(QWidget* parent)
  : BaseWidget(parent), ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
  ui_->group_box->setTitle("Reference X Direction");

  // Manually set x value
  ui_->double_spin_box_x->setValue(1.0);
}

void FixedOrientationModifierWidget::configure(const YAML::Node& config)
{
  auto dir = YAML::getMember<Eigen::Vector3d>(config, "ref_x_dir");
  dir.normalize();

  ui_->double_spin_box_x->setValue(dir.x());
  ui_->double_spin_box_y->setValue(dir.y());
  ui_->double_spin_box_z->setValue(dir.z());
}

void FixedOrientationModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "FixedOrientation";
  Eigen::Vector3d dir(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  config["ref_x_dir"] = dir;
}

}  // namespace noether
