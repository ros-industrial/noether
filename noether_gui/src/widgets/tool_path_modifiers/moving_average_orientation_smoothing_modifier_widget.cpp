#include <noether_gui/widgets/tool_path_modifiers/moving_average_orientation_smoothing_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/moving_average_orientation_smoothing_modifier.h>
#include <QFormLayout>
#include <QLabel>
#include <QSpinBox>
#include <yaml-cpp/yaml.h>

namespace noether
{
MovingAverageOrientationSmoothingModifierWidget::MovingAverageOrientationSmoothingModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
  , layout_(new QFormLayout(this))
  , label_(new QLabel("Window size", this))
  , window_size_(new QSpinBox(this))
{
  layout_->addRow(label_, window_size_);
  window_size_->setMinimum(3);
}

void MovingAverageOrientationSmoothingModifierWidget::configure(const YAML::Node& config)
{
  window_size_->setValue(getEntry<int>(config, "window_size"));
}

void MovingAverageOrientationSmoothingModifierWidget::save(YAML::Node& config) const
{
  config["window_size"] = window_size_->value();
}

ToolPathModifier::ConstPtr MovingAverageOrientationSmoothingModifierWidget::create() const
{
  return std::make_unique<MovingAverageOrientationSmoothingModifier>(window_size_->value());
}

}  // namespace noether
