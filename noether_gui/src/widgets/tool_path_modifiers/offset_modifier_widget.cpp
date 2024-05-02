#include <noether_gui/widgets/tool_path_modifiers/offset_modifier_widget.h>
#include "ui_vector3d_editor_widget.h"
#include "ui_quaternation_editor_widget.h"
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/offset_modifier.h>
#include <yaml-cpp/yaml.h>

namespace noether
{
OffsetModifierWidget::OffsetModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), ui_vector_(new Ui::Vector3dEditor()), ui_quaternion_(new Ui::QuaternionEditor())
{
  auto layout = new QVBoxLayout(this);

  // Print text on created widget
  auto page_vector = new QWidget(this);
  ui_vector_->setupUi(page_vector);
  ui_vector_->group_box->setTitle("local offset");
  layout->addWidget(page_vector);

  auto page_quaternion = new QWidget(this);
  ui_quaternion_->setupUi(page_quaternion);
  ui_quaternion_->group_box->setTitle("local rotation");
  layout->addWidget(page_quaternion);

  setLayout(layout);
}

void OffsetModifierWidget::configure(const YAML::Node& config)
{
  Eigen::Vector3d dir(getEntry<double>(config, "x"), getEntry<double>(config, "y"), getEntry<double>(config, "z"));
  dir.normalize();

  ui_vector_->double_spin_box_x->setValue(dir.x());
  ui_vector_->double_spin_box_y->setValue(dir.y());
  ui_vector_->double_spin_box_z->setValue(dir.z());

  Eigen::Quaterniond q(getEntry<double>(config, "qx"),
                       getEntry<double>(config, "qy"),
                       getEntry<double>(config, "qz"),
                       getEntry<double>(config, "qw"));

  ui_quaternion_->double_spin_box_qx->setValue(q.x());
  ui_quaternion_->double_spin_box_qy->setValue(q.y());
  ui_quaternion_->double_spin_box_qz->setValue(q.z());
  ui_quaternion_->double_spin_box_qw->setValue(q.w());
}

void OffsetModifierWidget::save(YAML::Node& config) const
{
  config["x"] = ui_vector_->double_spin_box_x->value();
  config["y"] = ui_vector_->double_spin_box_y->value();
  config["z"] = ui_vector_->double_spin_box_z->value();
  config["qx"] = ui_quaternion_->double_spin_box_qx->value();
  config["qy"] = ui_quaternion_->double_spin_box_qy->value();
  config["qz"] = ui_quaternion_->double_spin_box_qz->value();
  config["qw"] = ui_quaternion_->double_spin_box_qw->value();
}

ToolPathModifier::ConstPtr OffsetModifierWidget::create() const
{
  Eigen::Vector3d position(ui_vector_->double_spin_box_x->value(),
                           ui_vector_->double_spin_box_y->value(),
                           ui_vector_->double_spin_box_z->value());
  Eigen::Quaterniond q(ui_quaternion_->double_spin_box_qw->value(),
                       ui_quaternion_->double_spin_box_qx->value(),
                       ui_quaternion_->double_spin_box_qy->value(),
                       ui_quaternion_->double_spin_box_qz->value());

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = position;
  offset.linear() = q.toRotationMatrix();

  return std::make_unique<OffsetModifier>(offset);
}

}  // namespace noether
