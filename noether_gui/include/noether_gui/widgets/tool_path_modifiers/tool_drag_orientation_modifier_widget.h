#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
class AngleDoubleSpinBox;
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class ToolDragOrientationToolPathModifierWidget : public ToolPathModifierWidget
{
public:
  ToolDragOrientationToolPathModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  AngleDoubleSpinBox* angle_offset_;
  DistanceDoubleSpinBox* tool_radius_;
};

}  // namespace noether
