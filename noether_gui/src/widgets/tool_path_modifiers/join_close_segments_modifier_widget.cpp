#include <noether_gui/widgets/tool_path_modifiers/join_close_segments_modifier_widget.h>
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>
#include <QFormLayout>

namespace noether
{
JoinCloseSegmentsToolPathModifierWidget::JoinCloseSegmentsToolPathModifierWidget(QWidget* parent)
  : BaseWidget(parent), distance_(new DistanceDoubleSpinBox(this))
{
  distance_->setMinimum(0.0);
  distance_->setValue(0.010);
  distance_->setSingleStep(0.010);
  distance_->setDecimals(3);

  auto layout = new QFormLayout(this);
  layout->addRow("Distance", distance_);
}

void JoinCloseSegmentsToolPathModifierWidget::configure(const YAML::Node& config)
{
  distance_->setValue(YAML::getMember<double>(config, "distance"));
}

void JoinCloseSegmentsToolPathModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "JoinCloseSegments";
  config["distance"] = distance_->value();
}

}  // namespace noether
