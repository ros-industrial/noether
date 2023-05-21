#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/core/tool_path_modifier.h>

class QFormLayout;
class QLabel;
class QSpinBox;

namespace noether
{
class MovingAverageOrientationSmoothingModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  MovingAverageOrientationSmoothingModifierWidget(QWidget* parent = nullptr);
  ToolPathModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QFormLayout* layout_;
  QLabel* label_;
  QSpinBox* window_size_;
};

}  // namespace noether
