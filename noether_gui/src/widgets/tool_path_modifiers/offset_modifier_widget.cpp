#include <noether_gui/widgets/tool_path_modifiers/offset_modifier_widget.h>
#include "../ui_vector3d_editor_widget.h"
#include "../ui_quaternion_editor_widget.h"
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
  ui_vector_->group_box->setTitle("Translation");

  // Configure the unit combo box with different units; default to meters
  ui_vector_->unit_combo_box->clear();
  ui_vector_->unit_combo_box->addItem("mm (0.001)", 0.001);
  ui_vector_->unit_combo_box->addItem("cm (0.01)", 0.01);
  ui_vector_->unit_combo_box->addItem("m (1.0)", 1.0);
  ui_vector_->unit_combo_box->addItem("inches (0.0254)", 0.0254);
  ui_vector_->unit_combo_box->addItem("feet (0.3048)", 0.3048);
  ui_vector_->unit_combo_box->addItem("yards (0.9144)", 0.9144);
  ui_vector_->unit_combo_box->setCurrentIndex(2);  // Set default unit to meters
  ui_vector_->unit_combo_box->setToolTip("Select the unit for translation");
  ui_vector_->double_spin_box_x->setToolTip("X component of the translation vector");
  ui_vector_->double_spin_box_y->setToolTip("Y component of the translation vector");
  ui_vector_->double_spin_box_z->setToolTip("Z component of the translation vector");
  layout->addWidget(page_vector);

  auto page_quaternion = new QWidget(this);
  ui_quaternion_->setupUi(page_quaternion);
  ui_quaternion_->group_box->setTitle("Rotation");
  ui_quaternion_->double_spin_box_qx->setToolTip("X component of the quaternion");
  ui_quaternion_->double_spin_box_qy->setToolTip("Y component of the quaternion");
  ui_quaternion_->double_spin_box_qz->setToolTip("Z component of the quaternion");
  ui_quaternion_->double_spin_box_qw->setToolTip("W component of the quaternion");
  layout->addWidget(page_quaternion);

  setLayout(layout);
}

void OffsetModifierWidget::configure(const YAML::Node& config)
{
  ui_vector_->double_spin_box_x->setValue(getEntry<double>(config, "x"));
  ui_vector_->double_spin_box_y->setValue(getEntry<double>(config, "y"));
  ui_vector_->double_spin_box_z->setValue(getEntry<double>(config, "z"));
  ui_quaternion_->double_spin_box_qx->setValue(getEntry<double>(config, "qx"));
  ui_quaternion_->double_spin_box_qy->setValue(getEntry<double>(config, "qy"));
  ui_quaternion_->double_spin_box_qz->setValue(getEntry<double>(config, "qz"));
  ui_quaternion_->double_spin_box_qw->setValue(getEntry<double>(config, "qw"));
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

  // Scale the position vector by the unit multiplier
  double unit_multiplier = ui_vector_->unit_combo_box->currentData().toDouble();
  position *= unit_multiplier;

  // Normalize the quaternion in case the values are not unit length
  q.normalize();

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = position;
  offset.linear() = q.toRotationMatrix();

  return std::make_unique<OffsetModifier>(offset);
}

}  // namespace noether
