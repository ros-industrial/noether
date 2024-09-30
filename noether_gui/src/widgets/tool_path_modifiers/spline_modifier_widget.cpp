#include <noether_gui/widgets/tool_path_modifiers/spline_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/spline_modifier.h>
#include <QDoubleSpinBox>
#include <QFormLayout>

namespace noether
{
SplineModifierWidget::SplineModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), point_spacing_(new QDoubleSpinBox(this))
{
  auto* layout = new QFormLayout(this);

  point_spacing_->setMinimum(0.0);
  point_spacing_->setDecimals(3);
  point_spacing_->setValue(0.010);
  point_spacing_->setSingleStep(0.010);

  layout->addRow("Point Spacing (m)", point_spacing_);
}

ToolPathModifier::ConstPtr SplineModifierWidget::create() const
{
  return std::make_unique<SplineModifier>(point_spacing_->value());
}

void SplineModifierWidget::configure(const YAML::Node& config)
{
  point_spacing_->setValue(getEntry<double>(config, "point_spacing"));
}

void SplineModifierWidget::save(YAML::Node& config) const { config["point_spacing"] = point_spacing_->value(); }

}  // namespace noether
