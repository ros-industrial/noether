#include <noether_gui/widgets/blending_tool_path_modifier_widgets.h>
#include <noether_tpp/tool_path_modifiers/blending_modifiers.h>

#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QCheckBox>

namespace noether
{
AngledOrientationToolPathModifierWidget::AngledOrientationToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
{
  auto layout = new QFormLayout(this);

  // Angle of rotation about the y axis
  angle_offset_ = new QDoubleSpinBox(this);
  angle_offset_->setMinimum(0.0);
  angle_offset_->setSingleStep(1.0);
  angle_offset_->setValue(30.0);
  angle_offset_->setDecimals(3);
  layout->addRow(new QLabel("Angle offset (deg)", this), angle_offset_);

  // Radius of the tool
  tool_radius_ = new QDoubleSpinBox(this);
  tool_radius_->setMinimum(0.0);
  tool_radius_->setSingleStep(0.01);
  tool_radius_->setValue(0.1);
  tool_radius_->setDecimals(3);
  layout->addRow(new QLabel("Tool radius (m)", this), tool_radius_);

  // Flip sign
  flip_sign_ = new QCheckBox(this);
  //
  layout->addRow(new QLabel("Flip", this), flip_sign_);

  setLayout(layout);
}

ToolPathModifier::ConstPtr AngledOrientationToolPathModifierWidget::create() const
{
  return std::make_unique<AngledOrientationModifier>(angle_offset_->value() * M_PI / 180.0, tool_radius_->value(), flip_sign_->value());
}

}  // namespace noether
