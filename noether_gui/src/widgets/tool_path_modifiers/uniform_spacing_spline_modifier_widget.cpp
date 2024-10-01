#include <noether_gui/widgets/tool_path_modifiers/uniform_spacing_spline_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/uniform_spacing_spline_modifier.h>
#include <QDoubleSpinBox>
#include <QFormLayout>

namespace noether
{
UniformSpacingSplineModifierWidget::UniformSpacingSplineModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), point_spacing_(new QDoubleSpinBox(this))
{
  auto* layout = new QFormLayout(this);

  point_spacing_->setMinimum(0.0);
  point_spacing_->setDecimals(3);
  point_spacing_->setValue(0.010);
  point_spacing_->setSingleStep(0.010);

  layout->addRow("Point Spacing (m)", point_spacing_);
}

ToolPathModifier::ConstPtr UniformSpacingSplineModifierWidget::create() const
{
  return std::make_unique<UniformSpacingSplineModifier>(point_spacing_->value());
}

void UniformSpacingSplineModifierWidget::configure(const YAML::Node& config)
{
  point_spacing_->setValue(getEntry<double>(config, "point_spacing"));
}

void UniformSpacingSplineModifierWidget::save(YAML::Node& config) const
{
  config["point_spacing"] = point_spacing_->value();
}

}  // namespace noether
