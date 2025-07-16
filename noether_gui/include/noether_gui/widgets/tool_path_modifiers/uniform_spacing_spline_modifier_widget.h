#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class UniformSpacingSplineModifierWidget : public ToolPathModifierWidget
{
public:
  UniformSpacingSplineModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  DistanceDoubleSpinBox* point_spacing_;
};

}  // namespace noether
