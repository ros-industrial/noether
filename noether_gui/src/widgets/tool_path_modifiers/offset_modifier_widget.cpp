#include <noether_gui/widgets/tool_path_modifiers/offset_modifier_widget.h>
#include "ui_vector3d_editor_widget.h"
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/offset_modifier.h>
#include <yaml-cpp/yaml.h>

namespace noether
{
OffsetModifierWidget::OffsetModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), ui_(new Ui::Vector3dEditor())
{
  // Print text on created widget
  ui_->setupUi(this);
  ui_->group_box->setTitle("Local offset applied to each toolpath");
}

void OffsetModifierWidget::configure(const YAML::Node& config)
{
  Eigen::Vector3d dir(getEntry<double>(config, "x"), getEntry<double>(config, "y"), getEntry<double>(config, "z"));
  dir.normalize();

  ui_->double_spin_box_x->setValue(dir.x());
  ui_->double_spin_box_y->setValue(dir.y());
  ui_->double_spin_box_z->setValue(dir.z());
}

void OffsetModifierWidget::save(YAML::Node& config) const
{
  config["x"] = ui_->double_spin_box_x->value();
  config["y"] = ui_->double_spin_box_y->value();
  config["z"] = ui_->double_spin_box_z->value();
}

ToolPathModifier::ConstPtr OffsetModifierWidget::create() const
{
  Eigen::Vector3d offset(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  return std::make_unique<OffsetModifier>(offset);
}

}  // namespace noether
