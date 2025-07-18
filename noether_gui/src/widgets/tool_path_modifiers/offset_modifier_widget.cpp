#include <noether_gui/widgets/tool_path_modifiers/offset_modifier_widget.h>
#include "../ui_vector3d_editor_widget.h"
#include "../ui_quaternion_editor_widget.h"

#include <noether_tpp/serialization.h>

namespace noether
{
OffsetModifierWidget::OffsetModifierWidget(QWidget* parent)
  : BaseWidget(parent), ui_vector_(new Ui::Vector3dEditor()), ui_quaternion_(new Ui::QuaternionEditor())
{
  auto layout = new QVBoxLayout(this);

  // Print text on created widget
  auto page_vector = new QWidget(this);
  ui_vector_->setupUi(page_vector);
  ui_vector_->group_box->setTitle("Translation");
  layout->addWidget(page_vector);

  auto page_quaternion = new QWidget(this);
  ui_quaternion_->setupUi(page_quaternion);
  ui_quaternion_->group_box->setTitle("Rotation");
  layout->addWidget(page_quaternion);

  setLayout(layout);
}

void OffsetModifierWidget::configure(const YAML::Node& config)
{
  auto offset = YAML::getMember<Eigen::Isometry3d>(config, "offset");

  ui_vector_->double_spin_box_x->setValue(offset.translation().x());
  ui_vector_->double_spin_box_y->setValue(offset.translation().y());
  ui_vector_->double_spin_box_z->setValue(offset.translation().z());

  Eigen::Quaterniond q(offset.rotation());
  ui_quaternion_->double_spin_box_qx->setValue(q.x());
  ui_quaternion_->double_spin_box_qy->setValue(q.y());
  ui_quaternion_->double_spin_box_qz->setValue(q.z());
  ui_quaternion_->double_spin_box_qw->setValue(q.w());
}

void OffsetModifierWidget::save(YAML::Node& config) const
{
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();

  offset.translation().x() = ui_vector_->double_spin_box_x->value();
  offset.translation().y() = ui_vector_->double_spin_box_y->value();
  offset.translation().z() = ui_vector_->double_spin_box_z->value();

  Eigen::Quaterniond q;
  q.x() = ui_quaternion_->double_spin_box_qx->value();
  q.y() = ui_quaternion_->double_spin_box_qy->value();
  q.z() = ui_quaternion_->double_spin_box_qz->value();
  q.w() = ui_quaternion_->double_spin_box_qw->value();
  offset.rotate(q);

  config["name"] = "Offset";
  config["offset"] = offset;
}

}  // namespace noether
