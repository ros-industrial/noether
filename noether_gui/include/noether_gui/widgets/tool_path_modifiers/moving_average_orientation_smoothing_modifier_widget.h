#pragma once

#include <noether_gui/widgets.h>

class QFormLayout;
class QLabel;
class QSpinBox;

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class MovingAverageOrientationSmoothingModifierWidget : public BaseWidget
{
public:
  MovingAverageOrientationSmoothingModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QFormLayout* layout_;
  QLabel* label_;
  QSpinBox* window_size_;
};

}  // namespace noether
