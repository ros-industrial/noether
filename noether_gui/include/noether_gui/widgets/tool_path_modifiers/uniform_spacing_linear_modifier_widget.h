#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class UniformSpacingLinearModifierWidget : public ToolPathModifierWidget
{
public:
  UniformSpacingLinearModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  DistanceDoubleSpinBox* point_spacing_;
};

}  // namespace noether
