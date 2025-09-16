#include <noether_gui/widgets/tool_path_modifiers/uniform_spacing_linear_modifier_widget.h>
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>
#include <QFormLayout>

namespace noether
{
UniformSpacingLinearModifierWidget::UniformSpacingLinearModifierWidget(QWidget* parent)
  : BaseWidget(parent), point_spacing_(new DistanceDoubleSpinBox(this))
{
  auto* layout = new QFormLayout(this);

  point_spacing_->setMinimum(0.0);
  point_spacing_->setDecimals(3);
  point_spacing_->setValue(0.010);
  point_spacing_->setSingleStep(0.010);

  layout->addRow("Point Spacing", point_spacing_);
}

void UniformSpacingLinearModifierWidget::configure(const YAML::Node& config)
{
  point_spacing_->setValue(YAML::getMember<double>(config, "point_spacing"));
}

void UniformSpacingLinearModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "UniformSpacingLinear";
  config["point_spacing"] = point_spacing_->value();
}

}  // namespace noether
