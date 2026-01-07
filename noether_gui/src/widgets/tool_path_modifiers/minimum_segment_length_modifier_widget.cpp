#include <noether_gui/widgets/tool_path_modifiers/minimum_segment_length_modifier_widget.h>
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>
#include <QFormLayout>

namespace noether
{
MinimumSegmentLengthToolPathModifierWidget::MinimumSegmentLengthToolPathModifierWidget(QWidget* parent)
  : BaseWidget(parent), minimum_length_(new DistanceDoubleSpinBox(this))
{
  minimum_length_->setMinimum(0.0);
  minimum_length_->setValue(0.010);
  minimum_length_->setSingleStep(0.010);
  minimum_length_->setDecimals(3);

  auto layout = new QFormLayout(this);
  layout->addRow("Minimum length", minimum_length_);
}

void MinimumSegmentLengthToolPathModifierWidget::configure(const YAML::Node& config)
{
  minimum_length_->setValue(YAML::getMember<double>(config, "minimum_length"));
}

void MinimumSegmentLengthToolPathModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "MinimumSegmentLength";
  config["minimum_length"] = minimum_length_->value();
}

}  // namespace noether
